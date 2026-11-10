#include <waypoint_follower/waypointFollower.h>
#include <waypoint_follower/new_platform_control.h>

#include <float.h>

double WaypointFollower::pure_pursuit()
{
  if (waypoints_size_ <= 0) {
    return 0.0;
  }

  double final_ld = 0.0;
  int closest_index = find_cloestindex(cur_pose_back);
  if (closest_index < 0) {
    return 0.0;
  }

  target_index_ = closest_index;

  // Speed-scheduled lookahead, replacing the fixed per-mission-state value. A
  // lookahead that does not move with speed gives the same steering angle at
  // 6 km/h and 25 km/h, i.e. wildly different lateral acceleration, and a short
  // one at speed is what makes the car saw back and forth across the line.
  lookahead_dist_ = adaptiveLookahead();

  std_msgs::msg::Float64 ld_msg;
  ld_msg.data = lookahead_dist_;
  ld_dbg_pub_->publish(ld_msg);

  for (int i = closest_index; i < waypoints_size_; ++i) {
    double dist = calcPlaneDist(cur_pose_back, waypoints_[i].pose);
    if (dist >= lookahead_dist_) {
      target_index_ = i;
      final_ld = dist;
      break;
    }
  }

  if (final_ld <= 0.0) {
    target_index_ = waypoints_size_ - 1;
    final_ld = calcPlaneDist(cur_pose_back, waypoints_[target_index_].pose);
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = waypoints_[target_index_].pose.pose.position.x;
  marker.pose.position.y = waypoints_[target_index_].pose.pose.position.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  pp_target_pub->publish(marker);

  double target_x = waypoints_[target_index_].pose.pose.position.x;
  double target_y = waypoints_[target_index_].pose.pose.position.y;

  double cur_x = cur_pose_back.pose.position.x;
  double cur_y = cur_pose_back.pose.position.y;
  if (ref_enable_ && ref_offset_ != 0.0) {
    // The arc below is the rear axle's, but /odom and the path are the INS
    // point ref_offset_ ahead of it: move both back along the body, the car by
    // its own heading and the target by the heading it will have there.
    const double yaw = cur_course_ * M_PI / 180.0;
    cur_x -= ref_offset_ * std::cos(yaw);
    cur_y -= ref_offset_ * std::sin(yaw);
    const double target_yaw = bodyHeadingAt(target_index_);
    target_x -= ref_offset_ * std::cos(target_yaw);
    target_y -= ref_offset_ * std::sin(target_yaw);
    final_ld = std::hypot(target_x - cur_x, target_y - cur_y);
  }
  double dx = (target_x - cur_x);
  double dy = (target_y - cur_y);

  double temp_theta = (atan2(dy, dx)) * 180.0 / M_PI;

  double deg_alpha = cur_course_ - temp_theta;
  if (temp_theta * cur_course_ < 0) {
    if (cur_course_ < -90.0 && temp_theta > 0) {
      cout << "case 0 " << endl;
      deg_alpha = cur_course_ - temp_theta + 360.0;
    } else if (cur_course_ > 0 && temp_theta < -90.0) {
      cout << "case 1 " << endl;
      deg_alpha = cur_course_ - temp_theta - 360.0;
    }
  }
  double alpha = deg_alpha * M_PI / 180.0;

  if (final_ld > 20.0) final_ld = 20.0;
  if (final_ld < 2.0) final_ld = 2.0;

  double cur_steer = atan2((2.0 * WHEEL_BASE * sin(alpha)) / final_ld, 1.0);
  cur_steer = cur_steer * 180.0 / M_PI;
  // pp 조향 gain
  // cur_steer *= 1.3;

  cout << "LookaheadDist    : " << lookahead_dist_ << endl;

  return cur_steer;
}

/**
 * @brief Signed lateral offset of @p pose from the path, positive to the left.
 */
double WaypointFollower::signedCrossTrack(
  int min_idx, const geometry_msgs::msg::PoseStamped & pose) const
{
  const double dx = pose.pose.position.x - waypoints_[min_idx].pose.pose.position.x;
  const double dy = pose.pose.position.y - waypoints_[min_idx].pose.pose.position.y;
  const double ryaw = waypoints_[min_idx].ryaw;
  // Project the offset onto the path normal instead of taking the raw distance,
  // which loses the side and therefore the direction to steer.
  return -std::sin(ryaw) * dx + std::cos(ryaw) * dy;
}

/**
 * @brief Projects @p pose onto the nearest path segment.
 * @details The foot of the perpendicular gives a cross-track error that does
 * not step as the nearest waypoint changes, and the tangent is the chord
 * tangent of the two segment ends, interpolated to the projected point.
 * Repeated waypoints have no direction and are skipped. With
 * tangent_half_window_m_ <= 0 this is the previous nearest-waypoint + ryaw
 * geometry, kept as the rollback. A non-zero @p shift moves every waypoint
 * that far along the body heading there (bodyHeadingAt), i.e. onto the path
 * the front axle draws; the returned yaw stays the tangent of the recorded
 * path.
 * @return false on a non-finite pose or waypoint, or a path with no segment of
 * non-zero length; process() then commands a stop.
 */
bool WaypointFollower::projectOnPath(
  const geometry_msgs::msg::PoseStamped & pose, PathProjection & out, double shift)
{
  out = PathProjection{};
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  if (waypoints_size_ <= 0 || !std::isfinite(px) || !std::isfinite(py)) {
    return false;
  }

  if (tangent_half_window_m_ <= 0.0 && shift == 0.0) {
    const int i = find_cloestindex(pose);
    out.seg = i;
    out.x = waypoints_[i].pose.pose.position.x;
    out.y = waypoints_[i].pose.pose.position.y;
    out.cte = signedCrossTrack(i, pose);
    out.yaw = waypoints_[i].ryaw;
    return std::isfinite(out.cte) && std::isfinite(out.yaw);
  }

  double best = std::numeric_limits<double>::infinity();
  double seg_yaw = 0.0;
  struct Pt { double x, y; };
  const auto shifted = [&](int i) {
    const auto & w = waypoints_[i].pose.pose.position;
    if (shift == 0.0) {
      return Pt{w.x, w.y};
    }
    const double h = bodyHeadingAt(i);
    return Pt{w.x + shift * std::cos(h), w.y + shift * std::sin(h)};
  };
  Pt q = shifted(0);
  for (int i = 0; i + 1 < waypoints_size_; ++i) {
    const Pt p = q;
    q = shifted(i + 1);
    const double len = std::hypot(q.x - p.x, q.y - p.y);
    if (!std::isfinite(len)) {
      return false;
    }
    if (len < 1e-6) {
      continue;
    }
    const double tx = (q.x - p.x) / len;
    const double ty = (q.y - p.y) / len;
    const double along = std::clamp((px - p.x) * tx + (py - p.y) * ty, 0.0, len);
    const double fx = p.x + along * tx;
    const double fy = p.y + along * ty;
    const double d = std::hypot(px - fx, py - fy);
    if (d < best) {
      best = d;
      out.seg = i;
      out.u = along / len;
      out.x = fx;
      out.y = fy;
      out.cte = tx * (py - fy) - ty * (px - fx);
      seg_yaw = std::atan2(ty, tx);
    }
  }
  if (out.seg < 0) {
    return false;
  }

  const double t0 = pathTangent(out.seg);
  const double t1 = pathTangent(out.seg + 1);
  if (std::isfinite(t0) && std::isfinite(t1)) {
    const double yaw = t0 + out.u * std::atan2(std::sin(t1 - t0), std::cos(t1 - t0));
    out.yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  } else {
    out.yaw = seg_yaw;
  }
  return std::isfinite(out.cte) && std::isfinite(out.yaw);
}

/**
 * @brief Extended Stanley steering law, in degrees.
 * @details Follows Hoffmann et al. 2007 with the additions evaluated in
 * arXiv:2408.15152:
 *
 *   delta = k_ang*dpsi
 *         + atan( k_dist*e / (k_damp*v + k_soft) )
 *         + k_rate*(r_meas - r_path)
 *         + k_steer*(delta_prev_cmd - delta_meas)
 *
 * The previous implementation set the cross-track gain to the path curvature,
 * so on a straight (curvature ~ 0) the whole correction term vanished and the
 * controller could not pull itself back onto the line; it also divided by raw
 * speed, which explodes towards +-90 deg of steering at standstill.
 */
double WaypointFollower::stanly()
{
  if (waypoints_size_ <= 0) {
    return 0.0;
  }

  const double rad_car_course = cur_course_ * M_PI / 180.0;

  PathProjection proj;
  if (!projectOnPath(cur_pose_front, proj, stanleyPathShift())) {
    last_cte_ = std::numeric_limits<double>::quiet_NaN();
    return std::numeric_limits<double>::quiet_NaN();   // process() turns this into a stop
  }
  const double cross_err = proj.cte;
  last_cte_ = cross_err;

  const double v = speedMps();

  // Heading error, wrapped to (-pi, pi].
  const double heading_err = atan2(sin(rad_car_course - proj.yaw),
                                   cos(rad_car_course - proj.yaw));

  // Cross-track term. k_damp*v + k_soft keeps this finite at any speed, where
  // the old atan2(k*e, v) swung to +-90 deg as the vehicle came to a stop.
  const double cross_term = atan2(k_dist_ * cross_err, k_damp_ * v + k_soft_);

  // Both feedback terms need a live steering reading. On a dead serial link the
  // last angle stays frozen, and using it would add a constant bias.
  const bool fb_ok = isFeedbackFresh();

  // Yaw-rate damping, the derivative action of the loop: compare the turn the
  // wheels are actually describing against the turn the path asks for. Path
  // curvature is positive to the left; this file's convention is negative =
  // left, hence -v*k. The raw rk column is not used here: its point-to-point
  // noise alone put up to 11 deg of jitter on the command at 25 km/h.
  double rate_term = 0.0;
  if (fb_ok) {
    const double r_meas = v * std::tan(steerCtrl()) / WHEEL_BASE;
    const int k_idx = (proj.u < 0.5) ? proj.seg : proj.seg + 1;
    const double r_path = -v * pathCurvature(k_idx);
    rate_term = -k_rate_ * (r_meas - r_path);
  }

  // Actuator lag: the wheels have not reached the last command yet, so push on
  // the part that has not been delivered.
  const double lag_term = fb_ok ? k_steer_ * (prev_steer_cmd_rad_ - steerCtrl()) : 0.0;

  // Signs follow pure_pursuit's convention (path to the left -> negative), which
  // the platform layer then inverts; heading_err and cross_err are both positive
  // when the path lies to the right, so both terms enter positively.
  const double steer = k_ang_ * heading_err + cross_term + rate_term + lag_term;
  const double deg_steer = steer * 180.0 / M_PI;

  dbg_[DBG_HEADING_DEG] = k_ang_ * heading_err * 180.0 / M_PI;
  dbg_[DBG_CROSS_DEG] = cross_term * 180.0 / M_PI;
  dbg_[DBG_RATE_DEG] = rate_term * 180.0 / M_PI;
  dbg_[DBG_LAG_DEG] = lag_term * 180.0 / M_PI;

  // Own namespace: pure pursuit publishes its target on the same topic as
  // my_namespace/0, and sharing it made the two markers overwrite each other.
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "stanley_target";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = proj.x;
  marker.pose.position.y = proj.y;
  marker.pose.position.z = waypoints_[proj.seg].pose.pose.position.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  st_target_pub->publish(marker);

  return deg_steer;
}

/**
 * @brief Highest speed the path curvature ahead allows, in km/h.
 * @details Point-wise limit v = sqrt(a_lat_max / |k|) from the lateral
 * acceleration budget, then a backward pass so the car is already slowed by the
 * time it reaches the corner rather than braking inside it:
 *
 *     v[i] = min( v[i], sqrt( v[i+1]^2 + 2*a_dec*ds ) )
 *
 * This is the backward half of the standard forward-backward velocity profile
 * solver used in racing-line planners.
 */
double WaypointFollower::curvatureSpeedLimitKmh()
{
  if (waypoints_size_ <= 1) {
    return std::numeric_limits<double>::max();
  }

  const int start = std::clamp(find_cloestindex(cur_pose_back), 0, waypoints_size_ - 1);

  // Point-wise limits over the horizon: curvature, and (anticipate_mission_)
  // the mission speed of the state each waypoint belongs to, so the backward
  // pass below brakes for a slower state before the car gets there.
  std::vector<double> v_lim;
  std::vector<double> ds;
  v_lim.reserve(64);
  ds.reserve(64);

  double point_here = std::numeric_limits<double>::max();   // curvature only, at the rear axle
  double travelled = 0.0;
  for (int i = start; i < waypoints_size_; ++i) {
    const double k = std::fabs(pathCurvature(i));
    // A perfectly straight segment imposes no limit; clamp k so it stays finite.
    double v_point = std::sqrt(a_lat_max_ / std::max(k, 1e-3));
    if (i == start) {
      point_here = v_point;
    }
    if (anticipate_mission_) {
      v_point = std::min(v_point, missionSpeedKmh(waypoints_[i].mission_state) / 3.6);
    }
    v_lim.push_back(v_point);

    if (i + 1 < waypoints_size_) {
      const double step = calcPlaneDist(waypoints_[i].pose, waypoints_[i + 1].pose);
      ds.push_back(step);
      travelled += step;
    }
    if (travelled > profile_horizon_) {
      break;
    }
  }

  if (v_lim.empty()) {
    return std::numeric_limits<double>::max();
  }

  // Backward pass: propagate each corner's limit upstream at the braking rate.
  for (int i = static_cast<int>(v_lim.size()) - 2; i >= 0; --i) {
    const double step = (i < static_cast<int>(ds.size())) ? ds[i] : 0.0;
    const double reachable = std::sqrt(v_lim[i + 1] * v_lim[i + 1] + 2.0 * a_dec_max_ * step);
    v_lim[i] = std::min(v_lim[i], reachable);
  }

  // Speed lag compensation. A new speed command only takes effect after the
  // longitudinal lag, by which time the car has moved on, so command the limit
  // of the point it will be at then. Without this the car reached the tightest
  // corner of the default course at 17 km/h instead of 12 (4.5 m/s^2 lateral).
  size_t pick = 0;
  if (speed_lag_comp_sec_ > 0.0) {
    const double ahead = speedMps() * speed_lag_comp_sec_;
    double run = 0.0;
    while (pick + 1 < v_lim.size() && pick < ds.size() && run + ds[pick] <= ahead) {
      run += ds[pick];
      ++pick;
    }
  }

  // Look-behind hold: the limits of the points the rear axle has just driven
  // over stay in force for exit_hold_m_ more metres, so the car does not speed
  // up out of a corner while the wheels are still unwinding, and two corners
  // closer than that are taken as one. The odometer follows the measured pose;
  // a jump (GPS glitch, relocalisation) restarts the window, since what lies
  // behind is then unknown.
  double hold_mps = std::numeric_limits<double>::max();
  if (exit_hold_m_ > 0.0 && std::isfinite(point_here)) {
    hold_mps = holdMin(hold_hist_, point_here, exit_hold_m_);
  }

  last_curv_limit_kmh_ = std::min(v_lim[pick], hold_mps) * 3.6;
  return last_curv_limit_kmh_;
}

/**
 * @brief Speed multiplier that falls off as the tracking error grows.
 * @details Oscillation feeds on speed, so backing off when the car is already
 * off the line is the cheapest way to break the cycle. During a lane change the
 * error is large by design, so an allowance is subtracted after the path
 * switches; without it the car would crawl through every swap. The allowance
 * shrinks linearly to zero over the hold window, so it only covers the error
 * the swap itself created: if the car has not converged by the end, the normal
 * damping is back in full instead of switching on as a step.
 */
double WaypointFollower::cteSpeedFactor() const
{
  double cte = std::fabs(last_cte_);

  if (laneChangeActive() && lane_change_hold_sec_ > 0.0) {
    const double elapsed = (this->now() - lane_change_stamp_).seconds();
    const double left = std::clamp(1.0 - elapsed / lane_change_hold_sec_, 0.0, 1.0);
    cte = std::max(0.0, cte - lane_change_allowance_ * left);
  }

  if (cte <= cte_dead_) {
    return 1.0;
  }
  if (cte >= cte_full_) {
    return cte_min_factor_;
  }
  const double t = (cte - cte_dead_) / (cte_full_ - cte_dead_);
  return 1.0 - t * (1.0 - cte_min_factor_);
}

/**
 * @brief Records one sample for the oscillation detector.
 * @details The sample is the lateral acceleration the wheels are producing,
 * v^2*tan(delta)/L, from the measured wheel angle (the command when the
 * feedback is stale), low-passed over osc_lpf_sec_. The raw command is not
 * usable at speed: in closed-loop simulation at 25 km/h it jitters +-0.7-1 deg
 * from localisation noise while the car holds the line to 3 cm, i.e. +-0.5-0.7
 * m/s^2, and counting those flips cut the speed on 83 % of clean laps. The
 * actuator already smooths the measured angle, and with the filter on top it
 * was 2 laps in 385 (8 s window). With osc_min_lat_acc_ <= 0 the raw command
 * in degrees is recorded instead (previous behaviour).
 */
void WaypointFollower::updateSteerHistory(double steer_deg)
{
  const double t = this->now().seconds();
  double sample = steer_deg;
  if (osc_min_lat_acc_ > 0.0) {
    const double delta = isFeedbackFresh() ? steerCtrl() : steer_deg * M_PI / 180.0;
    const double v = speedMps();
    const double a_lat = v * v * std::tan(delta) / WHEEL_BASE;
    if (!std::isfinite(a_lat)) {
      return;   // keep a bad reading out of the filter state for good
    }
    const double dt = t - osc_lpf_t_;
    if (!osc_lpf_init_ || osc_lpf_sec_ <= 0.0 || dt <= 0.0 || dt > 0.5) {
      osc_lpf_val_ = a_lat;
    } else {
      osc_lpf_val_ += dt / (osc_lpf_sec_ + dt) * (a_lat - osc_lpf_val_);
    }
    osc_lpf_t_ = t;
    osc_lpf_init_ = true;
    sample = osc_lpf_val_;
  }
  steer_hist_.emplace_back(t, sample);
  while (!steer_hist_.empty() && (t - steer_hist_.front().first) > osc_window_sec_) {
    steer_hist_.pop_front();
  }
}

/**
 * @brief Speed multiplier that falls off when the steering starts hunting.
 * @details Counts sign flips of the detector signal (updateSteerHistory) above
 * a magnitude threshold. This separates true oscillation from a lane change: a
 * lane change is a large but one-sided excursion and produces few flips,
 * whereas hunting flips sign every half period. The window has to hold a few
 * periods of the slowest weave worth catching: the high-speed weave in
 * simulation ran at 0.12-0.5 Hz, which a 2 s window never saw more than two
 * flips of. A 0.33 Hz weave (0.5 s localisation lag at 25 km/h) gave only 4-5
 * flips in 6 s, right at the trigger, hence 8 s.
 */
double WaypointFollower::oscillationFactor()
{
  const double thr = (osc_min_lat_acc_ > 0.0) ? osc_min_lat_acc_ : osc_min_steer_deg_;
  int flips = 0;
  int last_sign = 0;
  for (const auto & s : steer_hist_) {
    if (std::fabs(s.second) < thr) {
      continue;
    }
    const int sign = (s.second > 0.0) ? 1 : -1;
    if (last_sign != 0 && sign != last_sign) {
      ++flips;
    }
    last_sign = sign;
  }

  if (flips < osc_flip_trigger_) {
    return 1.0;
  }
  // Ramp down over the next few flips rather than stepping, so the speed does
  // not itself start oscillating with the detector. The first cut comes at
  // osc_flip_trigger_ flips; it used to wait for one more.
  const double excess = static_cast<double>(flips - osc_flip_trigger_ + 1) / 4.0;
  const double factor = std::max(osc_min_factor_, 1.0 - excess * (1.0 - osc_min_factor_));
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "[waypoint_follower] steering hunting: %d flips in %.1f s, speed x%.2f",
    flips, osc_window_sec_, factor);
  return factor;
}

/**
 * @brief Applies the curvature profile and the tracking-error damping.
 * @return target speed in km/h, before the acceleration ramp
 */
double WaypointFollower::applySpeedLimits(double desired_kmh)
{
  // The curvature profile only ever caps the mission speed; it never raises it.
  const double curv_limit_kmh = curvatureSpeedLimitKmh();
  const double cte_factor = cteSpeedFactor();
  const double osc_factor = oscillationFactor();
  dbg_[DBG_CURV_LIMIT_KMH] = curv_limit_kmh;
  dbg_[DBG_CTE_FACTOR] = cte_factor;
  dbg_[DBG_OSC_FACTOR] = osc_factor;

  double v_kmh = std::min(desired_kmh, curv_limit_kmh);

  v_kmh *= cte_factor;
  v_kmh *= osc_factor;
  return v_kmh;
}

/**
 * @brief Acceleration ramp; the last thing applied before the command goes out.
 * @details It has to see every other cap (INS quality, odometry loss) or its
 * internal state runs ahead of what was actually sent: a 7 km/h INS cap held
 * for ten seconds would leave the ramp sitting at 15 km/h, and the speed would
 * step straight back up the instant INS recovered. Slowing down is never
 * rate-limited.
 */
double WaypointFollower::rampSpeed(double target_kmh)
{
  double v_kmh = target_kmh;
  const rclcpp::Time now_t = this->now();
  if (prev_cmd_stamp_.nanoseconds() != 0) {
    const double dt = (now_t - prev_cmd_stamp_).seconds();
    if (dt > 0.0 && dt < 0.5 && v_kmh > cmd_speed_kmh_) {
      v_kmh = std::min(v_kmh, cmd_speed_kmh_ + a_acc_max_ * dt * 3.6);
    }
  }
  prev_cmd_stamp_ = now_t;
  cmd_speed_kmh_ = v_kmh;
  return v_kmh;
}

/**
 * @brief Steering slew-rate limit; the steering counterpart of rampSpeed().
 * @details The steering actuator is a position servo with no published slew
 * spec, and neither the HLV<->PCU protocol nor serial_bridge.cpp limits how
 * far the target angle can jump between cycles. Without this, a noisy
 * heading/cross-track term can hand the actuator a fresh step every cycle;
 * oscillationFactor() only reacts after such a step has already gone out, so
 * this is the earlier, cheaper fix. Symmetric in both directions (unlike
 * rampSpeed, which only limits speeding up): a fast unwind is just as much a
 * step to the actuator as a fast wind-up. steer_rate_limit_deg_s_ <= 0 keeps
 * the previous unlimited behaviour.
 */
double WaypointFollower::rampSteer(double target_deg)
{
  double deg = target_deg;
  const rclcpp::Time now_t = this->now();
  if (steer_rate_limit_deg_s_ > 0.0 && steer_ramp_stamp_.nanoseconds() != 0) {
    const double dt = (now_t - steer_ramp_stamp_).seconds();
    if (dt > 0.0 && dt < 0.5) {
      const double max_step = steer_rate_limit_deg_s_ * dt;
      deg = std::clamp(
        deg, prev_steer_ramped_deg_ - max_step, prev_steer_ramped_deg_ + max_step);
    }
  }
  steer_ramp_stamp_ = now_t;
  prev_steer_ramped_deg_ = deg;
  return deg;
}

/**
 * @brief Remembers the steering that will actually reach the wheels.
 * @details The Stanley lag term compares this against the measured angle, so it
 * must be the delivered command: clipped to the +-20 deg the
 * actuator can reach, and zero when stopped (PlatformConnector centres the
 * wheel at zero speed). Comparing against an unclipped value keeps pushing
 * further into saturation.
 */
void WaypointFollower::recordDeliveredCommand(double steer_deg, double speed_kmh)
{
  double delivered = std::clamp(steer_deg, -19.9999, 19.9999);
  if (speed_kmh == 0.0) {
    delivered = 0.0;
  }
  prev_steer_cmd_rad_ = delivered * M_PI / 180.0;
  updateSteerHistory(delivered);
}

int WaypointFollower::find_cloestindex(geometry_msgs::msg::PoseStamped pose)
{
  double min_dis = DBL_MAX;
  int min_idx = 0;
  for (int i = 0; i < waypoints_size_; i++) {
    double dist = calcPlaneDist(pose, waypoints_[i].pose);
    if (dist < min_dis) {
      min_dis = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

double WaypointFollower::cross_error(geometry_msgs::msg::PoseStamped pose)
{
  double min_dis = DBL_MAX;
  int min_idx = 0;
  for (int i = 0; i < waypoints_size_; i++) {
    double dist = calcPlaneDist(pose, waypoints_[i].pose);
    if (dist < min_dis) {
      min_dis = dist;
      min_idx = i;
    }
  }
  (void)min_idx;
  return min_dis;
}

/**
 * @brief Lookahead distance scheduled on speed, Ld = ld_gain*v + ld_min.
 * @details The standard adaptive pure-pursuit law (see arXiv:2111.08873 and the
 * Coursera/Algorithms-for-Automated-Driving treatments): the lookahead has to
 * scale with speed or the controller is effectively a fixed-curvature follower.
 * Defaults span the range the per-state constants used to cover by hand.
 */
double WaypointFollower::adaptiveLookahead() const
{
  double ld = ld_gain_ * speedMps() + ld_min_;
  ld = std::clamp(ld, ld_min_, ld_max_);
  // Never exceed the value hand-tuned for this mission state; those encode
  // corner knowledge (4.75 m through the first tight bend) that a pure speed
  // schedule does not have.
  if (ld_state_cap_ > 0.0) {
    ld = std::min(ld, ld_state_cap_);
  }
  return ld;
}

/**
 * @brief Moves the per-state lookahead cap toward @p cap_target at a limited
 * rate [m/s] and returns the new cap.
 * @details The cap switches at a state boundary, and a pure-pursuit lookahead
 * that jumps with it moves the target point, and with it the steering, in a
 * single cycle. Only the cap is slowed: limiting the lookahead itself also held
 * back the speed schedule while accelerating (1.6 m per m/s at 1.5 m/s^2 is
 * 2.4 m/s), and the short lookahead at speed set off a weave on the start
 * straight in simulation with a slow localisation. Growth is slower than
 * shrinking, which keeps the lookahead short for a few seconds after a corner
 * so the steering does not unwind early. No cap (<= 0) moves toward
 * lookahead.max_m, which the schedule never exceeds anyway.
 */
/**
 * @brief Advances the odometer that the look-behind holds are measured on.
 * @details Travelled path of the measured rear-axle pose. A jump (GPS glitch,
 * relocalisation, > 2 m in one cycle) restarts both hold windows, since what
 * lies behind is then unknown; a NaN pose is skipped so it cannot poison the
 * odometer.
 */
void WaypointFollower::updateOdometer()
{
  const auto & p = cur_pose_back.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
    return;
  }
  if (odo_init_) {
    const double step = std::hypot(p.x - odo_prev_x_, p.y - odo_prev_y_);
    if (step > 2.0) {
      hold_hist_.clear();
      ld_hold_hist_.clear();
    } else {
      odo_m_ += step;
    }
  }
  odo_prev_x_ = p.x;
  odo_prev_y_ = p.y;
  odo_init_ = true;
}

/**
 * @brief Records @p value at the current odometer reading and returns the
 * smallest value seen over the last @p window metres.
 * @details Samples within 5 cm are merged (keeping the smaller), which bounds
 * the history at about window / 0.05 entries even while the car stands still.
 */
double WaypointFollower::holdMin(
  std::deque<std::pair<double, double>> & hist, double value, double window)
{
  if (!hist.empty() && odo_m_ - hist.back().first < 0.05) {
    hist.back().second = std::min(hist.back().second, value);
  } else {
    hist.emplace_back(odo_m_, value);
  }
  while (!hist.empty() && odo_m_ - hist.front().first > window) {
    hist.pop_front();
  }
  double m = value;
  for (const auto & h : hist) {
    m = std::min(m, h.second);
  }
  return m;
}

/**
 * @brief Lookahead cap from the path geometry, in m; 0 = no cap.
 * @details curv_coef * sqrt(R) for the tightest radius within the next
 * speed-scheduled lookahead of path, never below curv_min_m, held for
 * curv_hold_m after the rear axle has passed it. A pure-pursuit target that
 * far ahead cuts a bend of radius R by roughly Ld^2 / (8R), so the cap keeps
 * that cut about constant from the R 4.6 m bend (Ld ~5 m) to the rotary
 * (~8 m); the hold stops the lookahead stretching while the wheels are still
 * unwinding at the exit.
 */
double WaypointFollower::curvatureLdCap()
{
  if (ld_curv_coef_ <= 0.0 || waypoints_size_ < 2) {
    return 0.0;
  }
  const double preview = std::clamp(ld_gain_ * speedMps() + ld_min_, ld_min_, ld_max_);
  const int start = find_cloestindex(cur_pose_back);
  double kmax = 0.0;
  double run = 0.0;
  for (int i = start; i + 1 < waypoints_size_ && run <= preview; ++i) {
    const double k = std::fabs(pathCurvature(i));
    if (std::isfinite(k)) {
      kmax = std::max(kmax, k);
    }
    run += segmentLength(i);
  }
  double cap = (kmax > 1e-3) ? std::max(ld_curv_min_, ld_curv_coef_ / std::sqrt(kmax)) : ld_max_;
  // Never above ld_max: the recorded straights still read k ~0.015 (a "cap" of
  // ~19 m), and the rate limiter remembers the cap, so an unclamped one had to
  // wind down from 19 m at rate_down_mps before it could shorten anything --
  // ~2 s late for the first bend after a straight.
  cap = std::min(cap, ld_max_);
  if (ld_curv_hold_m_ > 0.0) {
    cap = holdMin(ld_hold_hist_, cap, ld_curv_hold_m_);
  }
  return cap;
}

double WaypointFollower::rateLimitCap(double cap_target)
{
  if (!(cap_target > 0.0)) {
    cap_target = ld_max_;
  }
  const rclcpp::Time now_t = this->now();
  double cap = cap_target;
  if (ld_state_cap_ > 0.0 && ld_stamp_.nanoseconds() != 0) {
    const double dt = (now_t - ld_stamp_).seconds();
    if (dt > 0.0 && dt < 0.5) {
      if (ld_rate_up_ > 0.0) {
        cap = std::min(cap, ld_state_cap_ + ld_rate_up_ * dt);
      }
      if (ld_rate_down_ > 0.0) {
        cap = std::max(cap, ld_state_cap_ - ld_rate_down_ * dt);
      }
    }
  }
  ld_stamp_ = now_t;
  return cap;
}

/**
 * @brief Blend of pure pursuit and Stanley, weighted by speed.
 * @details Pure pursuit stays in charge at low speed, where its geometric arc
 * handles tight corners gracefully and Stanley's cross-track term is jumpy.
 * Stanley is faded in as speed rises because it regulates the cross-track error
 * directly, which is what keeps the car off the oscillation that builds when a
 * pure-pursuit arc is chased at speed.
 */
double WaypointFollower::hybrid_control()
{
  if (waypoints_size_ <= 0) {
    return 0.0;
  }

  // Cross-track error is needed by the speed logic whether or not Stanley is
  // weighted in, so it is measured here rather than as a side effect of stanly().
  PathProjection proj;
  last_cte_ = projectOnPath(cur_pose_front, proj, stanleyPathShift()) ?
    proj.cte : std::numeric_limits<double>::quiet_NaN();

  // Published even when NaN: the gap in the plot is the projection failing,
  // which is also what makes process() command a stop.
  std_msgs::msg::Float64 cte_msg;
  cte_msg.data = last_cte_;
  cte_dbg_pub_->publish(cte_msg);

  const double v = speedMps();
  double ratio = (v - stanley_fade_lo_) / (stanley_fade_hi_ - stanley_fade_lo_);
  ratio = std::clamp(ratio, 0.0, 1.0) * stanley_max_ratio_;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  dbg_[DBG_HEADING_DEG] = dbg_[DBG_CROSS_DEG] = dbg_[DBG_RATE_DEG] = dbg_[DBG_LAG_DEG] = nan;

  const double pp = pure_pursuit();
  const double st = (ratio > 0.0) ? stanly() : 0.0;
  const double steer = (1.0 - ratio) * pp + ratio * st;

  dbg_[DBG_V_MPS] = v;
  dbg_[DBG_RATIO] = ratio;
  dbg_[DBG_PP_DEG] = pp;
  dbg_[DBG_ST_DEG] = (ratio > 0.0) ? st : nan;
  dbg_[DBG_BLEND_DEG] = steer;
  dbg_[DBG_CTE_M] = last_cte_;
  dbg_[DBG_LD_M] = lookahead_dist_;
  dbg_[DBG_LD_CAP_M] = ld_state_cap_;

  // prev_steer_cmd_rad_ and the oscillation history are updated in process(),
  // from the command that is actually sent (recordDeliveredCommand).
  return steer;
}
