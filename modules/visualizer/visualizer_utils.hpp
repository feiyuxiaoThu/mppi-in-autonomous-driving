/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:04:19
 * @LastEditTime: 2026-03-25
 * @FilePath: /mppi-in-autonomous-driving/modules/visualizer/visualizer_utils.hpp
 * Simplified visualization utilities without CommonRoad dependencies
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "foxglove/schemas.hpp"

#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

constexpr double kObstacleHeightVRU = 1.7;
constexpr double kObstacleHeightVehicle = 1.6;
constexpr double kObstacleHeightBus = 3.0;
constexpr foxglove::schemas::Color kObstacleColorVRU = {0.22, 0.43, 0.64, 0.8};      // #3A6EA5
constexpr foxglove::schemas::Color kObstacleColorBus = {0.8, 0.2, 0.2, 0.8};         // #CC3333
constexpr foxglove::schemas::Color kObstacleColorVehicle = {0.83, 0.59, 0.25, 0.8};  // #D4963F

inline foxglove::schemas::Quaternion yaw_to_quaternion(double yaw) {
  foxglove::schemas::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

inline foxglove::schemas::Pose construct_pose(void) {
  foxglove::schemas::Pose pose;
  pose.position = foxglove::schemas::Vector3{0.0, 0.0, 0.0};
  pose.orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0};
  return pose;
}

/**
 * @brief Get obstacle size and color based on obstacle type
 * @param obstacle_type Obstacle type as integer (0=VEHICLE, 1=PEDESTRIAN, etc.)
 * @param obstacle_length Obstacle length in meters
 * @param obstacle_width Obstacle width in meters
 * @return Tuple of (size vector, color)
 */
std::tuple<foxglove::schemas::Vector3, foxglove::schemas::Color> get_obstacle_size_and_color(
    int obstacle_type, double obstacle_length, double obstacle_width);

/**
 * @brief Create a line using mesh triangulation to avoid transparency issues
 *
 * This function creates a TriangleListPrimitive representing a line with given thickness.
 * Using mesh triangulation avoids the "sugar-coated haws" effect (darker color near points)
 * that occurs with LinePrimitive when transparency is less than 1.
 *
 * @param points Vector of 3D points defining the line path
 * @param thickness Line thickness (width perpendicular to line direction)
 * @param color Line color (RGBA)
 * @param gradient_fade If true, alpha fades from start to 0 with non-linear decay
 * @return foxglove::schemas::TriangleListPrimitive Triangle mesh representing the line
 */
foxglove::schemas::TriangleListPrimitive create_line_mesh(
    const std::vector<foxglove::schemas::Point3>& points, double thickness,
    const foxglove::schemas::Color& color, bool gradient_fade = false);