/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-30 00:00:00
 * @LastEditTime: 2026-03-30 00:00:00
 * @FilePath: /mppi-in-autonomous-driving/common/trajectory_utils.hpp
 * Copyright (c) 2026 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace common {

/**
 * @brief Normalize angle to [-pi, pi]
 */
inline double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief Resample a trajectory to match fixed time steps
 */
inline std::vector<PathPoint> ResampleTrajectoryToHorizon(
    std::vector<PathPoint> path, double dt, int horizon) {
  
  if (path.empty()) return {};

  // Ensure path is sorted by time to satisfy std::lower_bound's requirements for black-box E2E outputs
  std::sort(path.begin(), path.end(), [](const PathPoint& a, const PathPoint& b) {
    return a.t < b.t;
  });
  
  std::vector<PathPoint> resampled;
  resampled.reserve(horizon + 2);

  double start_t = path.front().t;
  for (int i = 0; i < horizon + 2; ++i) {
    double target_t = start_t + i * dt;
    
    // Find the two points in the original path that bracket target_t
    auto it = std::lower_bound(path.begin(), path.end(), target_t, 
                               [](const PathPoint& p, double t) { return p.t < t; });
    
    if (it == path.begin()) {
      resampled.push_back(path.front());
    } else if (it == path.end()) {
      resampled.push_back(path.back());
    } else {
      const auto& p1 = *(it - 1);
      const auto& p2 = *it;
      
      double dt_orig = p2.t - p1.t;
      if (std::abs(dt_orig) < 1e-6) {
        resampled.push_back(p1);
        continue;
      }
      
      double ratio = (target_t - p1.t) / dt_orig;
      ratio = std::max(0.0, std::min(1.0, ratio));
      
      PathPoint p;
      p.x = p1.x + ratio * (p2.x - p1.x);
      p.y = p1.y + ratio * (p2.y - p1.y);
      p.v = p1.v + ratio * (p2.v - p1.v);
      p.t = target_t;
      
      // Angle interpolation (handling wrap-around)
      double yaw_diff = NormalizeAngle(p2.yaw - p1.yaw);
      p.yaw = NormalizeAngle(p1.yaw + ratio * yaw_diff);
      
      resampled.push_back(p);
    }
  }
  
  return resampled;
}

/**
 * @brief Project state trajectory (x, y, yaw, v) to control prior (jerk, steer_rate)
 * 
 * @param path Input state trajectory
 * @param wheelbase Vehicle wheelbase
 * @param dt Control time step
 * @param horizon Planning horizon
 * @param max_jerk Max longitudinal jerk limit
 * @param max_steer_rate Max lateral steer rate limit
 * @param max_steer_angle Max steering angle limit (rad)
 */
inline std::vector<std::pair<double, double>> ProjectStateTrajectoryToControlPrior(
    const std::vector<PathPoint>& path, double wheelbase, double dt, int horizon,
    double max_jerk = 1.5, double max_steer_rate = 0.07, double max_steer_angle = 0.15) {
  
  std::vector<std::pair<double, double>> controls(horizon, {0.0, 0.0});
  if (path.size() < 3) return controls;

  // 1. Resample to ensure fixed dt
  auto resampled_path = ResampleTrajectoryToHorizon(path, dt, horizon);
  
  size_t n = resampled_path.size();
  std::vector<double> accels(n - 1, 0.0);
  std::vector<double> steers(n - 1, 0.0);

  // 2. Kinematic inversion (first derivative)
  for (size_t i = 0; i < n - 1; ++i) {
    accels[i] = (resampled_path[i+1].v - resampled_path[i].v) / dt;
    
    double yaw_dot = NormalizeAngle(resampled_path[i+1].yaw - resampled_path[i].yaw) / dt;
    double v = std::max(resampled_path[i].v, 0.5); // Minimum velocity for steering stability
    
    // delta = atan(yaw_dot * L / v)
    steers[i] = std::atan2(yaw_dot * wheelbase, v);
    
    // Clamp steering angle to real vehicle limits (e.g., ±0.15 rad)
    steers[i] = std::max(-max_steer_angle, std::min(max_steer_angle, steers[i]));
  }

  // 3. Control derivation (second derivative)
  for (size_t i = 0; i < static_cast<size_t>(horizon); ++i) {
    if (i + 1 < accels.size()) {
      double jerk = (accels[i+1] - accels[i]) / dt;
      double steer_rate = NormalizeAngle(steers[i+1] - steers[i]) / dt;
      
      // Use provided limits for clipping to ensure prior is within searchable control space
      jerk = std::max(-max_jerk, std::min(max_jerk, jerk));
      steer_rate = std::max(-max_steer_rate, std::min(max_steer_rate, steer_rate));
      
      controls[i] = {jerk, steer_rate};
    } else {
      controls[i] = {0.0, 0.0};
    }
  }

  return controls;
}

/**
 * @brief Deprecated: use ProjectStateTrajectoryToControlPrior for better robustness
 */
inline std::vector<std::pair<double, double>> TrajectoryToControl(
    const std::vector<PathPoint>& path, double wheelbase, double dt, int horizon) {
  return ProjectStateTrajectoryToControlPrior(path, wheelbase, dt, horizon);
}

} // namespace common
