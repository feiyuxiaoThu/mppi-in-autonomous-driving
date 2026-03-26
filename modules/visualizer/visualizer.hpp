/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-01-19 00:00:00
 * @LastEditTime: 2026-03-25
 * @FilePath: /mppi-in-autonomous-driving/modules/visualizer/visualizer.hpp
 * Simplified visualizer without CommonRoad dependencies
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/protos/config.pb.h"
#include "common/protos/planning_info.pb.h"
#include "common/reference_line.hpp"
#include "foxglove/foxglove.hpp"
#include "foxglove/mcap.hpp"
#include "foxglove/server.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

class Visualizer {
public:
  Visualizer() = delete;
  explicit Visualizer(const YAML::Node& config);
  ~Visualizer();

  // Initialization and lifecycle management
  bool start();
  void stop();

  // Publish visualization data
  void log_ego_state(const StateInfo& ego_state);
  void log_transform(const foxglove::schemas::FrameTransform& transform);
  void log_loop_runtime(double elapsed_seconds);
  void log_planning_info(const protos::planning::PlanningInfo& planning_info);
  void log_trajectory(const protos::planning::PlanningInfo& planning_info);
  void log_sampled_trajectories(const protos::planning::PlanningInfo& planning_info);
  void log_reference_line(const std::shared_ptr<ReferenceLine>& reference_line);
  void log_obstacles(const std::shared_ptr<common::ObstacleList>& obstacle_list,
                     const StateInfo& current_ego_state, double perception_range_m);
  void log_obstacle_predictions(const std::shared_ptr<common::ObstacleList>& obstacle_list,
                                const StateInfo& current_ego_state, double perception_range_m);
  void log_simulation_config(const protos::config::SimulationConfig& sim_config);

private:
  // Channel registration and initialization
  bool register_publish_channels();

  foxglove::Schema build_protobuf_schema(const google::protobuf::Descriptor* message_descriptor,
                                         std::vector<std::uint8_t>& schema_buffer) const;

  // Scene update generation methods
  foxglove::schemas::SceneUpdate get_ego_scene_update(const StateInfo& ego_state) const;
  foxglove::schemas::SceneUpdate get_trajectory_scene_update(
      const protos::planning::PlanningInfo& planning_info) const;
  foxglove::schemas::SceneUpdate get_sampled_scene_update(
      const protos::planning::PlanningInfo& planning_info) const;
  foxglove::schemas::SceneUpdate get_reference_line_scene_update(
      const std::shared_ptr<ReferenceLine>& reference_line);
  foxglove::schemas::SceneUpdate get_obstacle_list_scene_update(
      const std::shared_ptr<common::ObstacleList>& obstacle_list,
      const StateInfo& current_ego_state, double perception_range_m);
  foxglove::schemas::SceneUpdate get_prediction_scene_update(
      const std::shared_ptr<common::ObstacleList>& obstacle_list,
      const StateInfo& current_ego_state, double perception_range_m);

private:
  // Logger
  std::shared_ptr<spdlog::logger> logger_{nullptr};

  // Configuration
  bool save_mcap_{false};

  std::unique_ptr<foxglove::McapWriter> mcap_writer_{nullptr};
  std::unique_ptr<foxglove::WebSocketServer> socket_server_{nullptr};

  // Channels for different data types
  std::unique_ptr<foxglove::RawChannel> loop_runtime_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> ego_car_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> obstacle_list_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> obstacle_prediction_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> reference_line_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> trajectory_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> sampled_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::FrameTransformChannel> transform_channel_{nullptr};
  std::unique_ptr<foxglove::RawChannel> planning_info_channel_{nullptr};
  std::unique_ptr<foxglove::RawChannel> simulation_config_channel_{nullptr};

  std::vector<std::uint8_t> planning_info_schema_buffer_{};
  std::vector<std::uint8_t> simulation_config_schema_buffer_{};

  // State tracking
  std::atomic<bool> running_{false};
  std::atomic<bool> reference_line_initialized_{false};
};