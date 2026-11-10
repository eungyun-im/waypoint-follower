#include <waypoint_follower/waypointFollower.h>
#include <waypoint_follower/new_platform_control.h>

void WaypointFollower::process()
{
  if (is_pose_ && is_lane_) {
    is_control_ = true;

    // Measured pose, before the delay compensation below: the odometer is the
    // distance the car has actually travelled, and the look-behind holds are
    // measured on it.
    updateOdometer();

    if (current_mission_state_ < 0 || current_mission_state_ >= 16) {
      cerr << "!!!MISSION STATE ERROR!!!" << endl;
    }

    spd_state_ = missionSpeedKmh(current_mission_state_);
  }

  if (is_control_) {
    // Delay compensation: /odom describes where the car was delay_comp_sec_ ago,
    // so steer for the pose it has reached since, projected along the arc the
    // measured wheel angle is describing. Only the localisation latency belongs
    // here; the steering actuator lag is a first-order lag, not a dead time, and
    // projecting over it as well (0.35 s) made the cross-track error 2.5x worse
    // in closed-loop simulation. The real pose is restored afterwards, so the
    // speed logic below still works from the measurement.
    const auto measured_back = cur_pose_back;
    const auto measured_front = cur_pose_front;
    const double measured_course = cur_course_;
    if (delay_comp_sec_ > 0.0) {
      const double v = speedMps();
      const double yaw = cur_course_ * M_PI / 180.0;
      // steerCtrl() is negative for left; yaw rate is positive for left.
      const double steer_left = isFeedbackFresh() ? -steerCtrl() : 0.0;
      const double yaw_rate = v * std::tan(steer_left) / WHEEL_BASE;
      const double tau = delay_comp_sec_;
      const double yaw_mid = yaw + 0.5 * yaw_rate * tau;
      const double yaw_end = yaw + yaw_rate * tau;
      cur_pose_back.pose.position.x += v * tau * std::cos(yaw_mid);
      cur_pose_back.pose.position.y += v * tau * std::sin(yaw_mid);
      cur_course_ = std::atan2(std::sin(yaw_end), std::cos(yaw_end)) * 180.0 / M_PI;
      cur_pose_front = cur_pose_back;
      cur_pose_front.pose.position.x += frontOffset() * std::cos(yaw_end);
      cur_pose_front.pose.position.y += frontOffset() * std::sin(yaw_end);
    }

    // Lookahead cap: the per-state one (ldStateCapM, lookahead.state_cap_m) or
    // the curvature-linked one (curvatureLdCap), whichever is shorter, changed
    // at a limited rate. The per-state speed is missionSpeedKmh().
    //
    // Computed here, after the delay compensation, because curvatureLdCap()
    // searches the path forward from find_cloestindex(cur_pose_back) and
    // pure_pursuit() picks its target from the same pose. Run before the
    // compensation the cap looked ahead from where the car was one
    // delay_comp_sec ago -- 1.0 m behind the steering geometry at 25 km/h --
    // so a corner entered the cap's preview window that much late.
    double cap = ldStateCapM(current_mission_state_);
    const double curv_cap = curvatureLdCap();
    if (curv_cap > 0.0) {
      cap = (cap > 0.0) ? std::min(cap, curv_cap) : curv_cap;
    }
    ld_state_cap_ = rateLimitCap(cap);

    input_steer_ = hybrid_control();

    cur_pose_back = measured_back;
    cur_pose_front = measured_front;
    cur_course_ = measured_course;

    // A non-finite pose or path poisons every term below, and the +-20 deg clamp
    // in PlatformConnector compares false against NaN and would pass it on to
    // the vehicle. Centre the wheel and stop instead. Same for a path that has
    // gone quiet or has no segment: hybrid_control() returns 0 for an empty
    // one and leaves last_cte_ at its old value, so the car would drive
    // straight on at mission speed.
    const bool path_ok = isPathUsable();
    const bool geometry_ok =
      path_ok && std::isfinite(input_steer_) && std::isfinite(last_cte_);
    if (!geometry_ok) {
      input_steer_ = 0.0;
      last_cte_ = 0.0;
      // Bypass the slew limiter: a bad pose/path must centre the wheel
      // immediately, not ramp to it, and the next good cycle should not
      // inherit a stale rate-limit origin from before the fault.
      prev_steer_ramped_deg_ = 0.0;
      steer_ramp_stamp_ = this->now();
      if (!path_ok) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[waypoint_follower] /final_waypoint stale (> %.2f s) or under 2 points (%d); "
          "commanding stop.", path_timeout_, waypoints_size_);
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[waypoint_follower] non-finite steering or cross-track (bad pose or path); "
          "commanding stop.");
      }
    } else {
      // Slew-limit last, on the value actually about to be sent.
      input_steer_ = rampSteer(input_steer_);
    }

    // Start from the mission speed and let ACC cut it for the vehicle ahead.
    double desired_speed = spd_state_;

    if (is_acc) {
      if (acc_index > 200) {
        desired_speed = spd_state_;
      } else if (acc_index <= 200 && acc_index > 50) {
        const double min_index = 50.0;
        const double max_index = 200.0;
        const double mid_index = 125.0;
        const double sigmoid_gain = 0.04;
        const double sigmoid_min = 1.0 / (1.0 + exp(-sigmoid_gain * (min_index - mid_index)));
        const double sigmoid_max = 1.0 / (1.0 + exp(-sigmoid_gain * (max_index - mid_index)));
        const double sigmoid_now = 1.0 / (1.0 + exp(-sigmoid_gain * (acc_index - mid_index)));
        double speed_ratio = (sigmoid_now - sigmoid_min) / (sigmoid_max - sigmoid_min);
        if (speed_ratio < 0.0) speed_ratio = 0.0;
        else if (speed_ratio > 1.0) speed_ratio = 1.0;
        desired_speed = spd_state_ * speed_ratio;
      } else {
        // <= 100, including 0 and negative: an obstacle at the start of the
        // local path is at the car, so stop rather than hold mission speed.
        desired_speed = 0.0;
      }
    }

    // Curvature profile, cross-track and oscillation damping; only ever
    // downwards from whatever the mission state and ACC already agreed on.
    input_speed_ = applySpeedLimits(desired_speed);

    if (!isInsStatusGood()) {
      if (spd_state_ == 0.0 || input_speed_ == 0.0) {
        input_speed_ = 0.0;
      } else if (input_speed_ > 7.0) {
        input_speed_ = 7.0;
      }
    }

    // is_control_ latches on the first pose, so without this the loop would keep
    // steering toward a pose that stopped updating and hold the mission speed.
    if (!isOdomFresh()) {
      input_speed_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[waypoint_follower] /odom stale (> %.2f s); commanding stop.", odom_timeout_);
    }
    if (!geometry_ok) {
      input_speed_ = 0.0;
    }

    // Ramp last so its state follows what is actually sent.
    input_speed_ = rampSpeed(input_speed_);
    recordDeliveredCommand(input_steer_, input_speed_);

    dbg_[DBG_CMD_DEG] = input_steer_;
    dbg_[DBG_CMD_KMH] = input_speed_;
    dbg_[DBG_FB_STEER_DEG] =
      isFeedbackFresh() ? steerCtrl() * 180.0 / M_PI : std::numeric_limits<double>::quiet_NaN();
    dbg_[DBG_MISSION_STATE] = current_mission_state_;
    std_msgs::msg::Float64MultiArray terms_msg;
    terms_msg.data.assign(dbg_, dbg_ + DBG_COUNT);
    terms_dbg_pub_->publish(terms_msg);

    if (pc_ != nullptr) {
      pc_->setFromWF(input_speed_, input_steer_, current_mission_state_);
      pc_->inputSerial();
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[waypoint_follower] PlatformConnector is not set; ERP command not published.");
    }
  }

  is_pose_ = false;
  is_course_ = false;
}
