/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:04:19
 * @LastEditTime: 2026-03-25
 * @FilePath: /mppi-in-autonomous-driving/modules/visualizer/visualizer_utils.cpp
 * Simplified visualization utilities without CommonRoad dependencies
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "visualizer_utils.hpp"

// Obstacle type enumeration (matches common::ObstacleType)
enum class ObstacleType {
  VEHICLE = 0,
  PEDESTRIAN = 1,
  BICYCLE = 2,
  BUS = 3,
  TRUCK = 4,
  UNKNOWN = 5
};

std::tuple<foxglove::schemas::Vector3, foxglove::schemas::Color> get_obstacle_size_and_color(
    int obstacle_type, double obstacle_length, double obstacle_width) {
  foxglove::schemas::Vector3 size;
  foxglove::schemas::Color color;
  
  switch (static_cast<ObstacleType>(obstacle_type)) {
    case ObstacleType::PEDESTRIAN:
    case ObstacleType::BICYCLE:
      size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightVRU};
      color = kObstacleColorVRU;
      break;
    case ObstacleType::BUS:
    case ObstacleType::TRUCK:
      size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightBus};
      color = kObstacleColorBus;
      break;
    case ObstacleType::VEHICLE:
    case ObstacleType::UNKNOWN:
    default:
      size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightVehicle};
      color = kObstacleColorVehicle;
      break;
  }
  
  return {size, color};
}

foxglove::schemas::TriangleListPrimitive create_line_mesh(
    const std::vector<foxglove::schemas::Point3>& points, double thickness,
    const foxglove::schemas::Color& color, bool gradient_fade /* = false */) {
  foxglove::schemas::TriangleListPrimitive mesh;

  if (points.size() < 2) {
    return mesh;  // Need at least 2 points to draw a line
  }

  const double half_thickness = thickness * 0.5;
  std::vector<foxglove::schemas::Point3> left_vertices;
  std::vector<foxglove::schemas::Point3> right_vertices;
  std::vector<foxglove::schemas::Color> vertex_colors;

  // Generate vertices along the line with perpendicular offset
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& curr = points[i];
    foxglove::schemas::Vector3 direction{0.0, 0.0, 0.0};

    // Calculate tangent direction
    if (i == 0) {
      // First point: use direction to next point
      direction.x = points[i + 1].x - curr.x;
      direction.y = points[i + 1].y - curr.y;
      direction.z = points[i + 1].z - curr.z;
    } else if (i == points.size() - 1) {
      // Last point: use direction from previous point
      direction.x = curr.x - points[i - 1].x;
      direction.y = curr.y - points[i - 1].y;
      direction.z = curr.z - points[i - 1].z;
    } else {
      // Middle points: use average of incoming and outgoing directions
      double dx1 = curr.x - points[i - 1].x;
      double dy1 = curr.y - points[i - 1].y;
      double dz1 = curr.z - points[i - 1].z;
      double dx2 = points[i + 1].x - curr.x;
      double dy2 = points[i + 1].y - curr.y;
      double dz2 = points[i + 1].z - curr.z;
      direction.x = dx1 + dx2;
      direction.y = dy1 + dy2;
      direction.z = dz1 + dz2;
    }

    // Normalize direction
    double length = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                              direction.z * direction.z);
    if (length > 1e-10) {
      direction.x /= length;
      direction.y /= length;
      direction.z /= length;
    }

    // Calculate perpendicular offset (assuming line is in XY plane, perpendicular is in XY)
    foxglove::schemas::Vector3 perpendicular;
    perpendicular.x = -direction.y;
    perpendicular.y = direction.x;
    perpendicular.z = 0.0;

    // Create left and right vertices
    foxglove::schemas::Point3 left_vertex;
    left_vertex.x = curr.x + perpendicular.x * half_thickness;
    left_vertex.y = curr.y + perpendicular.y * half_thickness;
    left_vertex.z = curr.z;

    foxglove::schemas::Point3 right_vertex;
    right_vertex.x = curr.x - perpendicular.x * half_thickness;
    right_vertex.y = curr.y - perpendicular.y * half_thickness;
    right_vertex.z = curr.z;

    left_vertices.push_back(left_vertex);
    right_vertices.push_back(right_vertex);

    // Calculate color with gradient fade if enabled
    if (gradient_fade) {
      foxglove::schemas::Color vertex_color = color;
      // Non-linear fade: use quadratic function (t^2) for slow start, fast end
      double t = static_cast<double>(i) / static_cast<double>(points.size() - 1);
      double alpha_multiplier = 1.0 - (t * t);  // Quadratic decay
      vertex_color.a = color.a * alpha_multiplier;
      vertex_colors.push_back(vertex_color);
    }
  }

  // Create triangles connecting adjacent vertex pairs
  for (size_t i = 0; i < points.size() - 1; ++i) {
    // First triangle: left[i], right[i], left[i+1]
    mesh.points.push_back(left_vertices[i]);
    mesh.points.push_back(right_vertices[i]);
    mesh.points.push_back(left_vertices[i + 1]);

    // Second triangle: right[i], right[i+1], left[i+1]
    mesh.points.push_back(right_vertices[i]);
    mesh.points.push_back(right_vertices[i + 1]);
    mesh.points.push_back(left_vertices[i + 1]);

    // Add per-vertex colors if gradient fade is enabled
    if (gradient_fade) {
      // First triangle vertices
      mesh.colors.push_back(vertex_colors[i]);
      mesh.colors.push_back(vertex_colors[i]);
      mesh.colors.push_back(vertex_colors[i + 1]);

      // Second triangle vertices
      mesh.colors.push_back(vertex_colors[i]);
      mesh.colors.push_back(vertex_colors[i + 1]);
      mesh.colors.push_back(vertex_colors[i + 1]);
    }
  }

  // Set uniform color for the entire mesh if no gradient
  if (!gradient_fade) {
    mesh.color = color;
  }

  return mesh;
}