#ifndef WAYPOINTFOLLOWER_H
#define WAYPOINTFOLLOWER_H

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <iostream>
#include <vector>
#include <deque>
#include <numeric>
#include <algorithm>
#include <limits>
#include <math.h>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <waypoint_follower_msgs/msg/lane.hpp>
#include <waypoint_follower_msgs/msg/waypoint.hpp>
#include <waypoint_follower_msgs/msg/state.hpp>

#include <waypoint_follower_msgs/msg/lidar_topic_msg.hpp>
#include <waypoint_follower_msgs/msg/feedback.hpp>

using namespace std;

class PlatformConnector;

class WaypointFollower : public rclcpp::Node
{
private:
  int closest_waypoint_;
  bool is_course_gps_;
  double change_dist_;
  uint32_t ins_status_;
  rclcpp::Time last_ins_status_stamp_;

  double prev_lpf;
  vector<waypoint_follower_msgs::msg::Waypoint> waypoints_;
  double max_search_dist_;
  int loader_number_;
  int waypoint_min;

  double spd_state_;
  double dist_;
  int current_mission_state_;
  bool is_pose_;
  bool is_course_;
  bool is_lane_;
  bool is_control_;

  double input_speed_;
  double input_steer_;

  geometry_msgs::msg::PoseStamped cur_pose_front, cur_pose_back;
  const double LD_RATIO = 0.4;
  double lookahead_dist_;
  double cur_speed_;
  int waypoints_size_;
  int target_index_;
  double ego_cur_speed_;

  double prev_ryaw_;
  double prev_yaw_;
  double cur_course_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> lidar2map_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> lidarodom2map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path past_path_rviz_;

  int lane_number_;
  double brake_;

  const double WHEEL_BASE = 1.353;
  const double MAX_SEARCH_DIST = 10.0;
  const double MAX_SUM = 58.0;
  const double MIN_SUM = -150.0;

  double n_gps_start_sec_;
  double n_gps_during_sec_;
  double n_gps_start_sec2_;
  double ex_x_;
  double ex_y_;
  rclcpp::Time ex_stamp_;

  bool is_acc;
  int acc_index;

  // ── ERP42 feedback, used directly rather than inferred from odometry ───────
  // Wheel-encoder speed and the measured steering angle are far cleaner than
  // differentiating GPS odometry, and the steering reading is what lets the
  // controller see its own actuator lag.
  double fb_speed_mps_;
  double fb_steer_rad_;
  rclcpp::Time fb_stamp_;

  // ── Stanley gains (Hoffmann 2007 + the extended form in arXiv:2408.15152) ──
  // delta = k_ang*dpsi + atan(k_dist*e / (k_damp*v + k_soft))
  //         + k_rate*(r_meas - r_path) + k_steer*(delta_cmd - delta_meas)
  double k_ang_;     // heading error gain
  double k_dist_;    // cross-track gain; this is a TUNING constant, not curvature
  double k_soft_;    // keeps the denominator finite at standstill
  double k_damp_;    // scales the denominator with speed, calming high-speed gain
  double k_rate_;    // yaw-rate damping, the derivative term of the loop
  double k_steer_;   // compensates steering actuator lag using the feedback angle
  // Both feedback terms ride on the measured steering angle, which the ERP42
  // Racing protocol documents as accurate to only +-4% of the +-20 deg range
  // (+-0.8 deg): a noise floor to keep in mind before raising k_rate_/k_steer_
  // past their current small values, since raising them amplifies that noise
  // along with the real signal.

  // Path tangent for the heading term: chord over +-this many metres of arc. The
  // recorded ryaw, like the heading of any single 0.1 m segment, jitters by
  // ~1.7 deg (std) point to point, which Stanley passes straight into the
  // steering. 0 falls back to the nearest waypoint and its recorded ryaw.
  double tangent_half_window_m_;

  // /odom (and so the recorded path) is the INS position, ref_offset_ ahead of
  // the rear axle, not the rear axle itself. Pure pursuit's arc is derived for
  // the rear axle and Stanley's law for the front axle, so each is given its
  // own axle and the path that axle draws when /odom follows the recorded one.
  // false = previous behaviour (/odom taken as the rear axle).
  bool   ref_enable_;
  double ref_offset_;        // m, /odom ahead of the rear axle

  // Where a pose meets the path.
  struct PathProjection {
    int seg = -1;          // segment start; the segment runs seg -> seg + 1
    double u = 0.0;        // fraction along that segment, [0, 1]
    double x = 0.0;        // projected point
    double y = 0.0;
    double cte = 0.0;      // signed lateral offset of the pose, positive to the left
    double yaw = 0.0;      // path tangent at the projected point [rad]
  };

  // Stanley is faded in with speed; pure pursuit keeps the low-speed corners.
  double stanley_max_ratio_;
  double stanley_fade_lo_;   // m/s, pure pursuit only at or below
  double stanley_fade_hi_;   // m/s, full blend at or above

  // ── Speed-scheduled lookahead: Ld = clamp(ld_gain*v + ld_min, min, max) ────
  double ld_gain_;
  double ld_min_;
  double ld_max_;
  // Hand-tuned per-mission-state lookahead, kept as an upper bound so the speed
  // schedule can never look further ahead than the course designer intended and
  // start cutting the tight corners.
  double ld_state_cap_;
  // ld_state_cap_ is the rate-limited value; this is the table it moves toward.
  // Cap per state [m], indexed by mission state. A negative entry keeps the
  // built-in table (ldStateCapM), 0 = no cap for that state.
  std::vector<double> ld_cap_m_;
  // Rate limit on the per-state cap [m/s], so a cap that steps at a state
  // change (11 -> 4.75 m into the first tight bend) does not step the steering.
  // Only the cap moves at this rate; the speed schedule under it is left alone.
  // Up is slower than down: a lookahead that stretches at a corner exit unwinds
  // the steering early, and that exit is where the 25 km/h laps ran wide.
  // 0 = no limit in that direction.
  double ld_rate_up_;
  double ld_rate_down_;
  rclcpp::Time ld_stamp_;
  // Curvature-linked cap: Ld <= curv_coef * sqrt(R), R the tightest radius
  // within the next speed-scheduled Ld of path, kept for curv_hold_m after the
  // rear axle has passed it (the lookahead counterpart of speed.exit_hold_m).
  // With it one mission speed can run the whole course: the lookahead shortens
  // for a bend by itself instead of through the per-state table. It joins the
  // per-state cap (the shorter wins) and moves at the same rate limit. Closed-
  // loop sim, every state at 25 km/h and no per-state cap: speed-scheduled Ld
  // alone 3/22 laps ran >0.5 m off (11-12 m into the R4.6 bend), cap alone
  // 2/22, cap + 5 m hold 0/22. 0 = off.
  double ld_curv_coef_;
  double ld_curv_min_;
  double ld_curv_hold_m_;
  std::deque<std::pair<double, double>> ld_hold_hist_;   // (odometer m, cap m)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ld_dbg_pub_;
  // Signed cross-track error actually used by Stanley and the speed damping.
  // Without it the only way to see the tracking error is to record a bag and
  // run test/fit_from_bag.py afterwards, which makes gain tuning a round trip
  // per trial instead of something that can be watched live on rqt_plot.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cte_dbg_pub_;
  // Every term of the steering and speed law for one control cycle, so gains
  // can be fitted per speed band from a bag (test/fit_osc_from_bag.py) instead
  // of being inferred from the summed command. Index = position in
  // /wf_debug/terms; keep in step with TERMS in that script. Angles in deg,
  // this file's steering convention (negative = left). NaN = not computed
  // this cycle (the Stanley terms while its weight is 0).
  enum DbgTerm {
    DBG_V_MPS = 0, DBG_RATIO, DBG_PP_DEG, DBG_ST_DEG, DBG_BLEND_DEG, DBG_CMD_DEG,
    DBG_HEADING_DEG, DBG_CROSS_DEG, DBG_RATE_DEG, DBG_LAG_DEG, DBG_CTE_M,
    DBG_LD_M, DBG_LD_CAP_M, DBG_CURV_LIMIT_KMH, DBG_CTE_FACTOR, DBG_OSC_FACTOR,
    DBG_CMD_KMH, DBG_FB_STEER_DEG, DBG_MISSION_STATE, DBG_COUNT
  };
  double dbg_[DBG_COUNT];
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr terms_dbg_pub_;

  // ── Curvature speed profile: v_lim = sqrt(a_lat_max / |k|) ────────────────
  double a_lat_max_;      // lateral acceleration budget [m/s^2]
  double a_dec_max_;      // deceleration used to brake *before* a curve [m/s^2]
  double a_acc_max_;      // acceleration ramp out of a curve [m/s^2]
  double profile_horizon_;  // how far down the path to look [m]
  // Curvature is taken from the waypoint positions over +-this many metres. The
  // recorded rk column is too noisy to use point by point; 0 falls back to it.
  double curv_half_window_m_;
  // The car reaches a new speed command only after the longitudinal lag, by
  // which time it has moved on, so the limit is read this far ahead in time.
  double speed_lag_comp_sec_;
  // Look-behind: the curvature limit of a point is kept until the rear axle is
  // this far past it. Without it the profile re-accelerates the moment the
  // curvature falls, while the wheels are still unwinding, and back-to-back
  // corners (the tight bend into the rotary on 0.csv) grow that into a 1 m
  // excursion. 0 = off.
  double exit_hold_m_;
  std::deque<std::pair<double, double>> hold_hist_;   // (odometer m, limit m/s)
  double odo_m_;
  double odo_prev_x_;
  double odo_prev_y_;
  bool odo_init_;

  // Mission speed per state [km/h], indexed by mission state. A negative entry
  // keeps the built-in table (missionSpeedKmh).
  std::vector<double> state_kmh_;
  // One mission speed for every state 0..15 [km/h], taking precedence over
  // state_kmh and the table; the curvature profile slows the car for bends.
  // Negative = off.
  double uniform_kmh_;
  // Brake for a slower mission state before reaching it: each waypoint's
  // mission speed is folded into the curvature profile's backward pass.
  // Without it a state's cap only applied once the car was in that state, so a
  // 25 -> 8 km/h step was entered at ~20 km/h. false = previous behaviour.
  bool anticipate_mission_;

  // Localisation latency. The pose is projected forward by this much before
  // steering is computed, so the controller acts on where the car is now.
  double delay_comp_sec_;

  // ── Cross-track feedback on speed (oscillation suppression) ───────────────
  double cte_dead_;        // below this the tracking error costs nothing
  double cte_full_;        // at this error the speed is cut to cte_min_factor_
  double cte_min_factor_;
  // Oscillation is detected from steering sign flips rather than raw error, so a
  // lane change -- large but one-sided error -- does not look like oscillation.
  // A flip counts when the lateral acceleration the wheels are producing,
  // v^2*tan(delta)/L, swings past +-osc_min_lat_acc_. A fixed steering angle
  // does not scale with speed: 3 deg is 2 m/s^2 at 25 km/h, so a weave had to be
  // violent before it counted. osc_min_lat_acc_ <= 0 = the old steering-angle
  // threshold on the command (osc_min_steer_deg_), unfiltered.
  double osc_min_lat_acc_;
  double osc_lpf_sec_;        // low-pass on the lateral acceleration [s]
  double osc_min_steer_deg_;
  double osc_window_sec_;
  int    osc_flip_trigger_;
  double osc_min_factor_;
  double osc_lpf_val_;
  double osc_lpf_t_;
  bool   osc_lpf_init_;

  // ── Lane change (0.csv <-> 1.csv) ─────────────────────────────────────────
  // While switching lanes the cross-track error is large *on purpose*, so the
  // error-based slowdown has to be held off or the car crawls through every swap.
  double lane_change_hold_sec_;
  double lane_change_allowance_;
  int    prev_lane_number_;
  rclcpp::Time lane_change_stamp_;

  // /odom watchdog. is_control_ latches true once the first pose and path
  // arrive and is never cleared, so without this the follower keeps steering
  // and driving on a frozen pose after the odometry source dies.
  rclcpp::Time last_odom_stamp_;
  double odom_timeout_;

  // /final_waypoint watchdog, same reason: without it the follower keeps
  // driving on the last path after local_path_publisher stops sending (it
  // skips publishing when no closest waypoint is found), and an empty path
  // left the wheel centred at mission speed.
  rclcpp::Time last_path_stamp_;
  double path_timeout_;

  // Controller state
  double prev_steer_cmd_rad_;
  double cmd_speed_kmh_;
  rclcpp::Time prev_cmd_stamp_;

  // ── Steering slew-rate limit ───────────────────────────────────────────────
  // The ERP42 Racing steering actuator (ERP_Racing manual, sec. 1.1/10) is a
  // position-controlled electric motor: HLV sends a target angle and the
  // actuator's own servo loop decides how fast to get there. Neither the
  // HLV<->PCU protocol nor serial_bridge.cpp puts any slew limit on that
  // target, so a noisy heading/cross-track term can hand the actuator a fresh
  // step every 50 ms cycle -- exactly the kind of input the manual warns
  // against for the drive motor ("순간적으로 큰 명령...급출발", E-STOP
  // "급격하게...부하"). oscillationFactor() only reacts after such a step has
  // already gone out; this stops it leaving in the first place.
  // Default 90 deg/s is an untested, conservative starting point (no actuator
  // slew spec is published); MEASURE on the vehicle and raise/lower it once
  // real data is available. 0 = off (previous behaviour, no limit).
  double steer_rate_limit_deg_s_;
  double prev_steer_ramped_deg_;
  rclcpp::Time steer_ramp_stamp_;
  // (sec, lateral accel m/s^2), or (sec, steer deg) with osc_min_lat_acc_ <= 0
  std::deque<std::pair<double, double>> steer_hist_;
  double last_cte_;
  double last_curv_limit_kmh_;

  // Publishers
  rclcpp::Publisher<waypoint_follower_msgs::msg::State>::SharedPtr lane_number_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_front;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_back;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pp_target_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr st_target_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr course_pub;

  // Subscribers
  rclcpp::Subscription<waypoint_follower_msgs::msg::LidarTopicMsg>::SharedPtr lidar_topic_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<waypoint_follower_msgs::msg::Lane>::SharedPtr lane_sub_;
  rclcpp::Subscription<waypoint_follower_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr ins_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ego_speed_sub_;
  rclcpp::Subscription<waypoint_follower_msgs::msg::Feedback>::SharedPtr erp42_fb_sub_;

  std_msgs::msg::Float64 course_msg_;

  double start_sec_;
  double during_sec_;
  bool gear_flag;
  PlatformConnector * pc_{nullptr};

public:
  WaypointFollower()
    : Node("waypoint_follower"),
      last_ins_status_stamp_(0, 0, RCL_ROS_TIME),
      ex_stamp_(0, 0, RCL_ROS_TIME),
      fb_stamp_(0, 0, RCL_ROS_TIME),
      ld_stamp_(0, 0, RCL_ROS_TIME),
      lane_change_stamp_(0, 0, RCL_ROS_TIME),
      last_odom_stamp_(0, 0, RCL_ROS_TIME),
      last_path_stamp_(0, 0, RCL_ROS_TIME),
      prev_cmd_stamp_(0, 0, RCL_ROS_TIME),
      steer_ramp_stamp_(0, 0, RCL_ROS_TIME)
  {
    initSetup();
  }

  ~WaypointFollower() override {
    waypoints_.clear();
  }

  void initSetup()
  {
    gear_flag = false;
    start_sec_ = 0.0;
    during_sec_ = 0.0;
    change_dist_ = 0.0;
    ins_status_ = 0;
    closest_waypoint_ = 0;
    is_course_gps_ = false;

    prev_lpf = 0;
    loader_number_ = 0;
    max_search_dist_ = 10.0;
    waypoint_min = -1;

    spd_state_ = 0.0;
    dist_ = 100.0;
    current_mission_state_ = -1;
    is_pose_ = false;
    is_course_ = false;
    is_lane_ = false;
    is_control_ = false;

    input_speed_ = 0.0;
    input_steer_ = 0.0;

    lookahead_dist_ = 0.0;
    ld_state_cap_ = 0.0;
    waypoints_size_ = 0;
    target_index_ = 0;
    cur_speed_ = 0.0;
    ego_cur_speed_ = 0.0;

    prev_ryaw_ = 0.0;
    prev_yaw_ = 0.0;
    cur_course_ = 0.0;

    lane_number_ = 0;
    n_gps_start_sec_ = 0.0;
    n_gps_during_sec_ = 0.0;
    n_gps_start_sec2_ = 0.0;
    ex_x_ = 0.0;
    ex_y_ = 0.0;

    brake_ = 0;
    is_acc = false;
    acc_index = 0;

    fb_speed_mps_ = 0.0;
    fb_steer_rad_ = 0.0;
    prev_lane_number_ = -1;
    prev_steer_cmd_rad_ = 0.0;
    cmd_speed_kmh_ = 0.0;
    last_cte_ = 0.0;
    last_curv_limit_kmh_ = 0.0;
    odo_m_ = 0.0;
    odo_prev_x_ = 0.0;
    odo_prev_y_ = 0.0;
    odo_init_ = false;
    osc_lpf_val_ = 0.0;
    osc_lpf_t_ = 0.0;
    osc_lpf_init_ = false;
    prev_steer_ramped_deg_ = 0.0;

    // Publishers
    vis_pub_front      = create_publisher<visualization_msgs::msg::Marker>("front_pose", 1);
    vis_pub_back       = create_publisher<visualization_msgs::msg::Marker>("back_pose", 1);
    pp_target_pub      = create_publisher<visualization_msgs::msg::Marker>("target_pose", 1);
    st_target_pub      = create_publisher<visualization_msgs::msg::Marker>("target_pose", 1);
    lane_number_pub_   = create_publisher<waypoint_follower_msgs::msg::State>("lane_number_msg_", 1);
    course_pub         = create_publisher<std_msgs::msg::Float64>("course_path", 1);
    // The lookahead actually used, for bags and tuning.
    ld_dbg_pub_        = create_publisher<std_msgs::msg::Float64>("/wf_debug/lookahead", 1);
    // Signed cross-track error [m], positive when the path lies to the right.
    // The signal Stanley gain fitting is read off: rqt_plot /wf_debug/cte.
    cte_dbg_pub_       = create_publisher<std_msgs::msg::Float64>("/wf_debug/cte", 1);
    terms_dbg_pub_     = create_publisher<std_msgs::msg::Float64MultiArray>("/wf_debug/terms", 1);
    std::fill(dbg_, dbg_ + DBG_COUNT, std::numeric_limits<double>::quiet_NaN());

    // Subscribers
    // Best effort, newest sample only. The control loop drains its subscriptions
    // at 20 Hz (spin_some in new_main.cpp) while /odom comes at 50 Hz or more
    // from a RELIABLE depth-10 publisher. With a RELIABLE depth-1 subscription
    // about 1 run in 6 of the closed-loop simulation fell into a state where
    // /odom arrived only ~5 times a second and ~250 ms late for the whole run
    // (1.7 m at 25 km/h): 0.1-0.15 m offsets in every curve, a weave on the
    // straights, and 4 of 36 laps more than 0.5 m off. Best effort: 0 of 108.
    // keep_last(1) matters: SensorDataQoS's default depth of 5 would make each
    // spin take the OLDEST queued pose.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&WaypointFollower::OdomCallback, this, std::placeholders::_1));
    dist_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/change_dist", 1, std::bind(&WaypointFollower::ChangeDistCallback, this, std::placeholders::_1));
    ins_status_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/pva/ins_status", 1, std::bind(&WaypointFollower::InsStatusCallback, this, std::placeholders::_1));
    lane_sub_ = create_subscription<waypoint_follower_msgs::msg::Lane>(
      "/final_waypoint", 1, std::bind(&WaypointFollower::LaneCallback, this, std::placeholders::_1));
    state_sub_ = create_subscription<waypoint_follower_msgs::msg::State>(
      "/gps_state", 1, std::bind(&WaypointFollower::StateCallback, this, std::placeholders::_1));
    ego_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/ego_speed", 1, std::bind(&WaypointFollower::EgoSpeedCallback, this, std::placeholders::_1));
    lidar_topic_sub_ = create_subscription<waypoint_follower_msgs::msg::LidarTopicMsg>(
      "lidar_topic", 1, std::bind(&WaypointFollower::LidarLaneCallback, this, std::placeholders::_1));

    erp42_fb_sub_ = create_subscription<waypoint_follower_msgs::msg::Feedback>(
      "/erp42_racing/feedback", 1,
      std::bind(&WaypointFollower::Erp42FeedbackCallback, this, std::placeholders::_1));

    // Stanley. k_dist is the cross-track gain; the previous code used the path
    // curvature here, which collapses the term to zero on straights where the
    // cross-track correction is needed most.
    k_ang_   = declare_parameter<double>("stanley.k_ang", 1.0);
    k_dist_  = declare_parameter<double>("stanley.k_dist", 0.8);
    k_soft_  = declare_parameter<double>("stanley.k_soft", 1.0);
    k_damp_  = declare_parameter<double>("stanley.k_damp", 1.0);
    k_rate_  = declare_parameter<double>("stanley.k_rate", 0.05);
    k_steer_ = declare_parameter<double>("stanley.k_steer", 0.2);
    tangent_half_window_m_ = declare_parameter<double>("stanley.tangent_half_window_m", 1.0);

    ref_enable_ = declare_parameter<bool>("reference.enable", false);
    ref_offset_ = declare_parameter<double>("reference.offset_m", 0.0);
    if (!(ref_offset_ >= 0.0 && ref_offset_ <= WHEEL_BASE)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[waypoint_follower] reference.offset_m %.3f outside [0, %.3f]; reference correction off.",
        ref_offset_, WHEEL_BASE);
      ref_enable_ = false;
      ref_offset_ = 0.0;
    }

    // Conservative by default: pure pursuit still carries most of the command
    // until the Stanley sign conventions have been confirmed on the vehicle.
    stanley_max_ratio_ = declare_parameter<double>("stanley.max_ratio", 0.7);
    stanley_fade_lo_   = declare_parameter<double>("stanley.fade_lo_mps", 1.5);
    stanley_fade_hi_   = declare_parameter<double>("stanley.fade_hi_mps", 5.0);

    // Lookahead grows with speed: short and sharp when crawling, long and smooth
    // when quick. Defaults bracket the hand-tuned per-state values (4.75-11 m).
    ld_gain_ = declare_parameter<double>("lookahead.gain", 1.6);
    ld_min_  = declare_parameter<double>("lookahead.min_m", 4.0);
    ld_max_  = declare_parameter<double>("lookahead.max_m", 12.0);
    ld_cap_m_ = declare_parameter<std::vector<double>>(
      "lookahead.state_cap_m", std::vector<double>(16, -1.0));
    ld_rate_up_   = declare_parameter<double>("lookahead.rate_up_mps", 2.0);
    ld_rate_down_ = declare_parameter<double>("lookahead.rate_down_mps", 4.0);
    ld_curv_coef_   = declare_parameter<double>("lookahead.curv_coef", 0.0);
    ld_curv_min_    = declare_parameter<double>("lookahead.curv_min_m", 3.0);
    ld_curv_hold_m_ = declare_parameter<double>("lookahead.curv_hold_m", 5.0);

    // Curvature speed profile. 3.0 m/s^2 takes the tightest corner of the
    // default course (R=4.6 m) at about 13 km/h.
    a_lat_max_ = declare_parameter<double>("speed.a_lat_max", 3.0);
    a_dec_max_ = declare_parameter<double>("speed.a_dec_max", 1.5);
    a_acc_max_ = declare_parameter<double>("speed.a_acc_max", 1.5);
    profile_horizon_ = declare_parameter<double>("speed.profile_horizon_m", 25.0);
    curv_half_window_m_ = declare_parameter<double>("speed.curvature_half_window_m", 2.0);
    speed_lag_comp_sec_ = declare_parameter<double>("speed.lag_comp_sec", 0.55);
    exit_hold_m_ = declare_parameter<double>("speed.exit_hold_m", 5.0);

    uniform_kmh_ = declare_parameter<double>("mission.uniform_kmh", -1.0);
    state_kmh_ = declare_parameter<std::vector<double>>(
      "mission.state_kmh", std::vector<double>(16, -1.0));
    anticipate_mission_ = declare_parameter<bool>("speed.anticipate_mission", true);

    delay_comp_sec_ = declare_parameter<double>("delay_comp_sec", 0.1);

    cte_dead_       = declare_parameter<double>("cte.deadband_m", 0.25);
    cte_full_       = declare_parameter<double>("cte.full_m", 1.2);
    cte_min_factor_ = declare_parameter<double>("cte.min_factor", 0.5);

    osc_min_lat_acc_   = declare_parameter<double>("osc.min_lat_acc", 0.5);
    osc_lpf_sec_       = declare_parameter<double>("osc.lpf_sec", 0.3);
    osc_min_steer_deg_ = declare_parameter<double>("osc.min_steer_deg", 3.0);
    // 20 s / 3 flips (was 8 s / 4): the previous pair's detection floor of
    // ~0.25 Hz missed the 0.12 Hz weaves this file's own history records; see
    // the rationale in tracking_params.yaml.
    osc_window_sec_    = declare_parameter<double>("osc.window_sec", 20.0);
    osc_flip_trigger_  = declare_parameter<int>("osc.flip_trigger", 3);
    osc_min_factor_    = declare_parameter<double>("osc.min_factor", 0.6);

    lane_change_hold_sec_  = declare_parameter<double>("lane_change.hold_sec", 4.0);
    // Measured gap between 0.csv and 1.csv: 3.31 m median on ochang, 2.39 m on
    // default, so 3.5 m covers the whole swap at the moment it starts.
    lane_change_allowance_ = declare_parameter<double>("lane_change.allowance_m", 3.5);

    // PCU's own watchdog declares Auto_Fail (-> Full Braking) 0.3 s after the
    // HLV link goes quiet (ERP_Racing manual, sec. 5-2/4). Keeping this below
    // that means WF's own graceful stop (ramped, not Full Braking) always
    // gets there first; 0.5 s let the PCU's harder stop win instead.
    odom_timeout_ = declare_parameter<double>("odom_timeout_sec", 0.25);
    // local_path_publisher sends one path per /odom message, so the same bound.
    path_timeout_ = declare_parameter<double>("path_timeout_sec", 0.25);

    steer_rate_limit_deg_s_ = declare_parameter<double>("steer.rate_limit_deg_s", 90.0);

    tf_buffer_     = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_   = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_odom_       = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    lidar2map_     = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    lidarodom2map_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  void ChangeDistCallback(const std_msgs::msg::Float64::ConstSharedPtr dist_msg) {
    change_dist_ = dist_msg->data;
  }

  void InsStatusCallback(const std_msgs::msg::UInt32::ConstSharedPtr status_msg) {
    ins_status_ = status_msg->data;
    last_ins_status_stamp_ = this->now();
  }

  void EgoSpeedCallback(const std_msgs::msg::Float32::ConstSharedPtr speed_msg) {
    ego_cur_speed_ = speed_msg->data;
  }

  void Erp42FeedbackCallback(const waypoint_follower_msgs::msg::Feedback::ConstSharedPtr msg) {
    fb_speed_mps_ = std::max(0.0, msg->speed);
    fb_steer_rad_ = msg->steering;
    fb_stamp_ = this->now();
  }

  bool isFeedbackFresh() const {
    return fb_stamp_.nanoseconds() != 0 && (this->now() - fb_stamp_).seconds() < 0.3;
  }

  /**
   * @brief Speed used by every controller term, in m/s.
   * @details The wheel encoder is the trustworthy source; GPS-differentiated
   * odometry is only a fallback for when the serial link is quiet.
   */
  double speedMps() const {
    return isFeedbackFresh() ? fb_speed_mps_ : cur_speed_;
  }

  /**
   * @brief Measured steering angle in this file's steering convention.
   * @details No sign change is needed: PlatformConnector inverts on the way out
   * (ctrl_msg.steering = -cmd_steer_rad) and serial_bridge inverts again on TX
   * (RAD2BYTE < 0) but not on RX (BYTE2RAD > 0), so the feedback angle already
   * comes back in the controller's own convention. A command of -10 deg reads
   * back as -10 deg once the wheels get there.
   */
  double steerCtrl() const {
    return fb_steer_rad_;
  }

  /** @brief Length of the path segment from waypoint @p j to @p j + 1. */
  double segmentLength(int j) const {
    const auto & p = waypoints_[j].pose.pose.position;
    const auto & q = waypoints_[j + 1].pose.pose.position;
    return std::hypot(q.x - p.x, q.y - p.y);
  }

  /**
   * @brief Waypoints [@p a, @p c] spanning @p w metres of arc either side of @p i.
   * @details Measured along the path, so it does not depend on the point
   * spacing. Near either end of the local path the window slides inwards rather
   * than shrinking; @p back and @p fwd are the arc lengths on each side of @p i.
   */
  void arcWindow(int i, double w, int & a, int & c, double & back, double & fwd) const {
    const int n = waypoints_size_;
    a = i;
    back = 0.0;
    while (a > 0 && back < w) { back += segmentLength(a - 1); --a; }
    c = i;
    fwd = 0.0;
    while (c < n - 1 && fwd < w) { fwd += segmentLength(c); ++c; }
    while (c < n - 1 && back + fwd < 2.0 * w) { fwd += segmentLength(c); ++c; }
    while (a > 0 && back + fwd < 2.0 * w) { back += segmentLength(a - 1); --a; }
  }

  /**
   * @brief Signed curvature of the path at waypoint @p i (+ = left).
   * @details Circle through the waypoints curv_half_window_m_ behind and ahead.
   * The recorded rk wobbles by about R = 5 m from point to point (the heading of
   * 0.1 m-spaced GPS points jitters by ~1 deg), which reads as a sharp bend on
   * a straight; three points 2 m apart average that away while keeping the real
   * corners (R 4.6 m on the default course).
   */
  double pathCurvature(int i) const {
    const int n = waypoints_size_;
    if (curv_half_window_m_ <= 0.0 || n < 3) {
      return waypoints_[i].rk;
    }
    int a, c;
    double back, fwd;
    arcWindow(i, curv_half_window_m_, a, c, back, fwd);

    // Middle point by arc length, which is i itself unless the window slid.
    int b = a;
    double half = 0.5 * (back + fwd), run = 0.0;
    while (b < c && run + segmentLength(b) <= half) { run += segmentLength(b); ++b; }
    if (b == a || b == c) {
      return 0.0;
    }

    const auto & A = waypoints_[a].pose.pose.position;
    const auto & B = waypoints_[b].pose.pose.position;
    const auto & C = waypoints_[c].pose.pose.position;
    const double cross = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
    const double denom = std::hypot(B.x - A.x, B.y - A.y) *
                         std::hypot(C.x - B.x, C.y - B.y) *
                         std::hypot(C.x - A.x, C.y - A.y);
    return denom > 1e-9 ? 2.0 * cross / denom : 0.0;
  }

  /**
   * @brief Path heading at waypoint @p i [rad], NaN if the window has no length.
   * @details Direction of the chord across tangent_half_window_m_ either side.
   * A symmetric chord is exactly tangent on a circular arc, and 2 m of it
   * averages away the GPS jitter that a single 0.1 m segment carries.
   */
  double pathTangent(int i) const {
    int a, c;
    double back, fwd;
    arcWindow(i, tangent_half_window_m_, a, c, back, fwd);
    const auto & A = waypoints_[a].pose.pose.position;
    const auto & C = waypoints_[c].pose.pose.position;
    if (std::hypot(C.x - A.x, C.y - A.y) < 1e-6) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::atan2(C.y - A.y, C.x - A.x);
  }

  // /odom to the Stanley front axle [m].
  double frontOffset() const { return ref_enable_ ? WHEEL_BASE - ref_offset_ : WHEEL_BASE; }
  // How far along the body the path Stanley tracks is moved [m].
  double stanleyPathShift() const { return ref_enable_ ? WHEEL_BASE - ref_offset_ : 0.0; }

  /**
   * @brief Heading of the car body when /odom sits on waypoint @p i [rad].
   * @details A point ref_offset_ ahead of the rear axle travels asin(d*k) to
   * the inside of the body heading (k = curvature of that point's path, + =
   * left), so the body points that much outside the path tangent. The axles
   * lie along this direction, not along the tangent. Falls back to the
   * adjacent segment where pathTangent() has no length.
   */
  double bodyHeadingAt(int i) const {
    double t = pathTangent(i);
    if (!std::isfinite(t)) {
      const int j = std::clamp(i, 0, std::max(0, waypoints_size_ - 2));
      const auto & p = waypoints_[j].pose.pose.position;
      const auto & q = waypoints_[std::min(j + 1, waypoints_size_ - 1)].pose.pose.position;
      t = std::atan2(q.y - p.y, q.x - p.x);
    }
    const double k = pathCurvature(i);
    const double slip = std::isfinite(k) ? std::asin(std::clamp(ref_offset_ * k, -0.5, 0.5)) : 0.0;
    return t - slip;
  }

  /**
   * @brief Mission speed [km/h] of mission state @p state.
   * @details The built-in table below unless mission.state_kmh gives that state
   * a non-negative value. Unknown states get a cautious 6 km/h.
   */
  double missionSpeedKmh(int state) const {
    static constexpr double kTable[16] = {
      15.0,   // 0  처음 직진
      8.0,    // 1  처음 작은 곡선
      15.0,   // 2  로터리 전
      12.0,   // 3  로터리
      11.0,   // 4  로터리 탈출 직진
      7.0,    // 5  방지턱 코너
      15.0,   // 6  코너 후 직진
      8.0,    // 7  처음 큰 곡선
      15.0,   // 8  사거리 전 직진
      11.0,   // 9  사거리 + 두번째 큰 곡선
      15.0,   // 10 사거리 탈출
      8.0,    // 11 세번째 곡선
      12.0,   // 12 세번째 곡선 탈출
      6.0,    // 13
      15.0,   // 14
      15.0,   // 15
    };
    if (state < 0 || state >= 16) {
      return 6.0;
    }
    if (uniform_kmh_ >= 0.0) {
      return uniform_kmh_;
    }
    if (state < static_cast<int>(state_kmh_.size()) && state_kmh_[state] >= 0.0) {
      return state_kmh_[state];
    }
    return kTable[state];
  }

  /**
   * @brief Lookahead cap [m] of mission state @p state; 0 = no cap.
   * @details The built-in table below unless lookahead.state_cap_m gives that
   * state a non-negative value. Unknown states get 5.25 m.
   *
   * The caps bind above ~5-9 km/h in most states, so the speed schedule only
   * really runs in 0/6/14/15. Raising the straight states 2/4/8/10 (9/8/11/11
   * m, or no cap) was tried in closed-loop simulation up to 25 km/h: no state
   * oscillated either way, since Stanley carries 70 % of the command at that
   * speed, but the corner after each got 3-7 cm worse and removing the caps
   * gave a 1.1 m excursion under slow steering + localisation lag. Hence the
   * table is unchanged.
   */
  double ldStateCapM(int state) const {
    static constexpr double kTable[16] = {
      11.0,   // 0  처음 직진
      4.75,   // 1  처음 작은 곡선
      6.0,    // 2  로터리 전
      6.0,    // 3  로터리
      5.25,   // 4  로터리 탈출 직진
      5.25,   // 5  방지턱 코너
      11.0,   // 6  코너 후 직진
      6.0,    // 7  처음 큰 곡선
      8.0,    // 8  사거리 전 직진
      6.0,    // 9  사거리 + 두번째 큰 곡선
      8.0,    // 10 사거리 탈출
      5.5,    // 11 세번째 곡선
      6.0,    // 12 세번째 곡선 탈출
      5.25,   // 13
      11.0,   // 14
      11.0,   // 15
    };
    if (state < 0 || state >= 16) {
      return 5.25;
    }
    if (state < static_cast<int>(ld_cap_m_.size()) && ld_cap_m_[state] >= 0.0) {
      return ld_cap_m_[state];
    }
    return kTable[state];
  }

  bool isOdomFresh() const {
    return last_odom_stamp_.nanoseconds() != 0 &&
           (this->now() - last_odom_stamp_).seconds() < odom_timeout_;
  }

  /** @brief A path arrived recently and has at least one segment to follow. */
  bool isPathUsable() const {
    return waypoints_size_ >= 2 && last_path_stamp_.nanoseconds() != 0 &&
           (this->now() - last_path_stamp_).seconds() < path_timeout_;
  }

  /** @brief True while the active path has recently switched between 0.csv/1.csv. */
  bool laneChangeActive() const {
    if (lane_change_stamp_.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - lane_change_stamp_).seconds() < lane_change_hold_sec_;
  }

  double calcPlaneDist(const geometry_msgs::msg::PoseStamped pose1,
                       const geometry_msgs::msg::PoseStamped pose2) {
    return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                pow(pose1.pose.position.y - pose2.pose.position.y, 2));
  }

  void getClosestWaypoint(geometry_msgs::msg::PoseStamped current_pose) {
    if (!waypoints_.empty()) {
      float dist_min = max_search_dist_;
      for (size_t i = 0; i < waypoints_.size(); i++) {
        float dist = calcPlaneDist(current_pose, waypoints_[i].pose);
        if (dist < dist_min) {
          dist_min = dist;
          waypoint_min = i;
        }
      }
      closest_waypoint_ = waypoint_min;
    } else {
      cout << "------ NO CLOSEST WAYPOINT -------" << endl;
    }
  }

  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    // Receive time, not the header stamp: the watchdog asks "has anything
    // arrived lately", which a stale or clock-skewed stamp cannot answer.
    last_odom_stamp_ = this->now();
    cur_pose_back.header = odom_msg->header;
    cur_pose_back.pose = odom_msg->pose.pose;

    tf2::Quaternion q(
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z,
      odom_msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    cur_course_ = yaw * 180.0 / M_PI;
    is_course_ = true;

    const double front_offset = frontOffset();
    cur_pose_front = cur_pose_back;
    cur_pose_front.pose.position.x = cur_pose_back.pose.position.x + front_offset * std::cos(yaw);
    cur_pose_front.pose.position.y = cur_pose_back.pose.position.y + front_offset * std::sin(yaw);

    rclcpp::Time stamp(odom_msg->header.stamp);
    double dt = 0.0;
    if (ex_stamp_.nanoseconds() > 0) dt = (stamp - ex_stamp_).seconds();

    const double SOME_MIN = 0.005;
    const double SOME_MAX = 0.2;

    if (ex_stamp_.nanoseconds() > 0 && dt > SOME_MIN && dt < SOME_MAX) {
      const double dx = cur_pose_back.pose.position.x - ex_x_;
      const double dy = cur_pose_back.pose.position.y - ex_y_;
      cur_speed_ = std::sqrt(dx * dx + dy * dy) / dt;
    } else {
      const double v_twist = odom_msg->twist.twist.linear.x;
      if (std::isfinite(v_twist) && v_twist >= 0.0) cur_speed_ = v_twist;
    }

    ex_x_ = cur_pose_back.pose.position.x;
    ex_y_ = cur_pose_back.pose.position.y;
    ex_stamp_ = stamp;

    is_pose_ = true;

    course_msg_.data = cur_course_;
    course_pub->publish(course_msg_);
  }

  void StateCallback(const waypoint_follower_msgs::msg::State::ConstSharedPtr state_msg) {
    dist_ = state_msg->dist;
    current_mission_state_ = state_msg->current_state;

    // A change of active path is the lane change itself; stamp it so the
    // cross-track slowdown knows the error it is about to see is deliberate.
    if (prev_lane_number_ >= 0 && state_msg->lane_number != prev_lane_number_) {
      lane_change_stamp_ = this->now();
      // Both look-behind holds are keyed on the odometer, and a lane change
      // swaps the path without moving the car, so updateOdometer()'s jump
      // guard never fires: the old path's curvature limits would otherwise
      // stay in force for exit_hold_m / curv_hold_m more metres of the new
      // one. What lies behind belongs to a path no longer being followed.
      hold_hist_.clear();
      ld_hold_hist_.clear();
      RCLCPP_INFO(this->get_logger(), "[wf] lane change %d -> %d",
                  prev_lane_number_, state_msg->lane_number);
    }
    prev_lane_number_ = state_msg->lane_number;
    loader_number_ = state_msg->lane_number;
  }

  void LaneCallback(const waypoint_follower_msgs::msg::Lane::ConstSharedPtr lane_msg) {
    waypoints_.clear();
    vector<waypoint_follower_msgs::msg::Waypoint>().swap(waypoints_);
    waypoints_ = lane_msg->waypoints;
    waypoints_size_ = waypoints_.size();
    // Receive time, as for /odom. An empty path still counts as "arrived";
    // isPathUsable() rejects it by size.
    last_path_stamp_ = this->now();
    if (waypoints_size_ != 0) is_lane_ = true;
  }

  void PathCallback(const nav_msgs::msg::Path::ConstSharedPtr path_msg) {
    waypoints_.clear();
    std::vector<waypoint_follower_msgs::msg::Waypoint>().swap(waypoints_);

    int index = 0;
    for (const auto& pose_stamped : path_msg->poses) {
      waypoint_follower_msgs::msg::Waypoint wp;
      wp.waypoint_index = index++;
      wp.pose = pose_stamped;
      waypoints_.push_back(wp);
    }
    waypoints_size_ = waypoints_.size();
    last_path_stamp_ = this->now();
    is_lane_ = !waypoints_.empty();
  }

  void LidarLaneCallback(const waypoint_follower_msgs::msg::LidarTopicMsg::ConstSharedPtr msg) {
    is_acc = msg->acc_flag;
    acc_index = msg->acc_index_gap;
  }

  void process();
  bool isInsStatusGood() const {
    if (last_ins_status_stamp_.nanoseconds() == 0) {
      return false;
    }
    return ins_status_ == 3 && (this->now() - last_ins_status_stamp_).seconds() < 0.5;
  }

  void setPlatformConnector(PlatformConnector * pc) { pc_ = pc; }
  double getSpeed() { return input_speed_; }
  double getSteer() { return input_steer_; }
  int get_Mission_State() { return current_mission_state_; }

  double hybrid_control();
  double pure_pursuit();
  double stanly();
  int find_cloestindex(geometry_msgs::msg::PoseStamped pose);
  double cross_error(geometry_msgs::msg::PoseStamped pose);

  double adaptiveLookahead() const;
  double rateLimitCap(double cap_target);
  double curvatureLdCap();
  void   updateOdometer();
  double holdMin(std::deque<std::pair<double, double>> & hist, double value, double window);
  double signedCrossTrack(int min_idx, const geometry_msgs::msg::PoseStamped & pose) const;
  bool projectOnPath(
    const geometry_msgs::msg::PoseStamped & pose, PathProjection & out, double shift = 0.0);
  double curvatureSpeedLimitKmh();
  double cteSpeedFactor() const;
  double oscillationFactor();
  void   updateSteerHistory(double steer_deg);
  double applySpeedLimits(double desired_kmh);
  double rampSpeed(double target_kmh);
  double rampSteer(double target_deg);
  void   recordDeliveredCommand(double steer_deg, double speed_kmh);
};

#endif
