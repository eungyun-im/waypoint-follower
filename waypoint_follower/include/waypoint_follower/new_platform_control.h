#ifndef PLATFORMCONTROL_H
#define PLATFORMCONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <limits>

#include <waypoint_follower_msgs/msg/control_command.hpp>
#include <waypoint_follower_msgs/msg/feedback.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std;

class PlatformConnector
{
private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<waypoint_follower_msgs::msg::Feedback>::SharedPtr erp42_status_sub_;
  rclcpp::Publisher<waypoint_follower_msgs::msg::ControlCommand>::SharedPtr erp42_ctrl_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ego_speed_pub_;

  // Vehicle speed limit; keep in step with serial_bridge max_speed_mps (6.94).
  static constexpr double MAX_SPEED_KMH = 25.0;

  const double GEAR_RATIO = 5.0;
  const double WHEEL_RADIOUS = 0.29;

  // Inputs from path planner
  double input_speed_;   // km/h (positive forward, negative requests reverse)
  double input_steer_;   // degrees
  double cur_speed_;     // km/h, fed from feedback
  double spd_state_;

  int   get_control_mode; // 0 = auto, 1 = manual (mapped from msg.manual_mode)
  bool  get_e_stop;
  int   get_gear;
  double feedback_speed_mps_;
  double get_steer;       // rad
  int   get_brake;
  int   get_encoder;
  int   get_alive;

  bool is_erp_state_;

  // Internal command state. ControlCommand only carries
  // speed/steering/brake; e_stop and gear are kept for legacy readers but not transmitted.
  bool   e_stop;
  int    gear;
  double cmd_speed_mps;  // m/s, sent to driver
  double cmd_steer_rad;  // rad, sent to driver
  int    brake;

  int count_;
  double prev_spd_state_;
  int current_mission_state_;
  std_msgs::msg::Float32 erp_speed_msgs_;

  double feedback_speed_kmh_;

  static double kmhToMps(double speed_kmh) {
    return speed_kmh / 3.6;
  }

  static double mpsToKmh(double speed_mps) {
    return speed_mps * 3.6;
  }

public:
  explicit PlatformConnector(rclcpp::Node * node) {
    initSetup(node);
  }
  ~PlatformConnector() = default;

  void initSetup(rclcpp::Node * node) {
    node_ = node;

    erp42_status_sub_ = node_->create_subscription<waypoint_follower_msgs::msg::Feedback>(
      "/erp42_racing/feedback", 10,
      std::bind(&PlatformConnector::ErpCallback, this, std::placeholders::_1));

    erp42_ctrl_cmd_pub_ = node_->create_publisher<waypoint_follower_msgs::msg::ControlCommand>(
      "/erp42_racing/control_command", rclcpp::QoS(1));
    ego_speed_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/ego_speed", 1);

    is_erp_state_ = false;
    e_stop = 0;
    gear = 0;
    cmd_speed_mps = 0.0;
    cmd_steer_rad = 0.0;
    brake = 1;
    count_ = 0;
    prev_spd_state_ = 0.0;
    current_mission_state_ = 0;
    feedback_speed_kmh_ = 0.0;
    input_speed_ = 0.0;
    input_steer_ = 0.0;
    cur_speed_ = 0.0;
    spd_state_ = 0.0;
    get_control_mode = 0;
    get_e_stop = false;
    get_gear = 0;
    feedback_speed_mps_ = 0.0;
    get_steer = 0.0;
    get_brake = 0;
    get_encoder = 0;
    get_alive = 0;
    RCLCPP_INFO(node_->get_logger(), "[PLATFORM_CONTROL] : INITIALIZED");
  }

  void inputSerial() {
    if (is_erp_state_) {
      cur_speed_ = feedback_speed_kmh_;
    }

    // input_speed_ : km/h, input_steer_ : deg
    if (input_speed_ == 0.0) {
      cmd_speed_mps = 0.0;   // brake is set to 99 below, with the steering reset
    } else if (input_speed_ > 0.0 && input_speed_ <= MAX_SPEED_KMH) {
      gear = 0;
      cmd_speed_mps = kmhToMps(input_speed_);
      brake = 1;
    } else if (input_speed_ > MAX_SPEED_KMH) {
      // Saturate rather than fall back to a lower speed: an overshooting request
      // must never make the car slower than a request just under the limit.
      // serial_bridge clamps again to its own max_speed_mps.
      gear = 0;
      cmd_speed_mps = kmhToMps(MAX_SPEED_KMH);
      brake = 1;
    } else if (input_speed_ > -20.0 && input_speed_ < 0.0) {
      // Reverse cannot be commanded over ControlCommand.
      // Use the erp42_racing/mode_command service (gear=REVERSE) if needed.
      gear = 2;
      cmd_speed_mps = 0.0;
      brake = 1;
      cout << "errorBBBBBBBack (reverse not commandable via ControlCommand)" << endl;
    } else {
      gear = 1;
      cmd_speed_mps = 0.0;
      brake = 1;
    }

    if (input_speed_ != 0.0) count_++;

    if (brake > 100) brake = 100;

    // Steering: clamp ±19.9999 deg → rad
    double steer_deg = input_steer_;
    if (steer_deg > 19.9999) steer_deg = 19.9999;
    else if (steer_deg < -19.9999) steer_deg = -19.9999;
    cmd_steer_rad = steer_deg * M_PI / 180.0;

    if (input_speed_ == 0.0) {
      cmd_steer_rad = 0.0;
      brake = 99;
    }

    pub_to_erp();
  }

  void pub_to_erp() {
    waypoint_follower_msgs::msg::ControlCommand ctrl_msg;
    ctrl_msg.speed = cmd_speed_mps;
    ctrl_msg.steering = -cmd_steer_rad;
    ctrl_msg.brake = static_cast<uint8_t>(brake);
    erp42_ctrl_cmd_pub_->publish(ctrl_msg);
  }

  void setFromWF(double wf_speed, double wf_steer, int _current_mission_state) {
    this->input_speed_ = wf_speed;
    this->input_steer_ = wf_steer;
    this->current_mission_state_ = _current_mission_state;
    spd_state_ = wf_speed;
  }

  double KMH_to_RPM(double InPut_speed_) {
    return (InPut_speed_ * GEAR_RATIO * 60.0) / (2.0 * WHEEL_RADIOUS * M_PI * 3.6);
  }

  void ErpCallback(const waypoint_follower_msgs::msg::Feedback::ConstSharedPtr msg) {
    get_control_mode = msg->manual_mode ? 1 : 0;
    get_e_stop = msg->emergency_stop;
    get_gear = msg->gear;
    feedback_speed_mps_ = msg->speed;
    get_steer = msg->steering;     // rad
    get_brake = msg->brake;
    get_encoder = msg->encoder_count;
    get_alive = msg->heartbeat;

    double prev_erp_speed_kmh = feedback_speed_kmh_;
    feedback_speed_kmh_ = mpsToKmh(feedback_speed_mps_);

    erp_speed_msgs_.data = feedback_speed_kmh_;
    ego_speed_pub_->publish(erp_speed_msgs_);

    if (prev_erp_speed_kmh == feedback_speed_kmh_) {
      feedback_speed_kmh_ = prev_erp_speed_kmh;
      is_erp_state_ = false;
    } else {
      is_erp_state_ = true;
    }
  }
};
#endif
