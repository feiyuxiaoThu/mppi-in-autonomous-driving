/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-01-19 00:00:00
 * @LastEditTime: 2026-03-25
 * @FilePath: /mppi-in-autonomous-driving/modules/visualizer/visualizer.cpp
 * Simplified visualizer without CommonRoad dependencies
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "visualizer.hpp"
#include "visualizer_utils.hpp"

#include <google/protobuf/descriptor.pb.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>

using namespace foxglove::schemas;

Visualizer::Visualizer(const YAML::Node& config) {
  std::string visualizer_log_level = "info";
  if (config["visualizer"]) {
    auto visualizer_config = config["visualizer"];
    save_mcap_ = visualizer_config["save_mcap"].as<bool>(true);
    visualizer_log_level = visualizer_config["log_level"].as<std::string>("info");
  } else {
    save_mcap_ = true;
  }

  logger_ = create_logger("visualizer_logger", visualizer_log_level);
}

Visualizer::~Visualizer() { stop(); }

bool Visualizer::start() {
  if (running_) {
    return true;
  }

  if (!register_publish_channels()) {
    LOG_ERROR(logger_, "Failed to register publish channels.");
    return false;
  }

  if (save_mcap_) {
    foxglove::McapWriterOptions mcap_options = {};
    std::filesystem::path mcap_save_dir = std::filesystem::path(PROJECT_SOURCE_DIR) / "logs";
    if (!std::filesystem::exists(mcap_save_dir)) {
      if (std::filesystem::create_directories(mcap_save_dir)) {
        LOG_INFO(logger_, "Created mcap save directory: {}", mcap_save_dir.string());
      } else {
        LOG_ERROR(logger_, "Failed to create mcap save directory: {}", mcap_save_dir.string());
        return false;
      }
    }
    std::string mcap_file_path = mcap_save_dir / ("mppi_" + TimeUtil::NowTimeString() + ".mcap");
    mcap_options.path = mcap_file_path;
    mcap_options.compression = foxglove::McapCompression::Lz4;
    auto writer_result = foxglove::McapWriter::create(mcap_options);
    if (!writer_result.has_value()) {
      LOG_ERROR(logger_, "Failed to create writer: {}", foxglove::strerror(writer_result.error()));
      return false;
    }
    mcap_writer_ = std::make_unique<foxglove::McapWriter>(std::move(writer_result.value()));
  }

  running_ = true;
  LOG_INFO(logger_, "Visualizer started.");
  return true;
}

void Visualizer::stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  if (mcap_writer_) {
    foxglove::FoxgloveError err = mcap_writer_->close();
    if (err != foxglove::FoxgloveError::Ok) {
      LOG_ERROR(logger_, "Failed to close writer: {}", foxglove::strerror(err));
    } else {
      LOG_INFO(logger_, "mcap file saved.");
    }
  }
  LOG_INFO(logger_, "Visualizer stopped.");
}

bool Visualizer::register_publish_channels() {
  using foxglove::RawChannel;
  using foxglove::WebSocketServer;

  foxglove::setLogLevel(foxglove::LogLevel::Error);
  foxglove::WebSocketServerOptions ws_options;
  ws_options.host = "0.0.0.0";
  ws_options.port = 8765;

  auto server_result = WebSocketServer::create(std::move(ws_options));
  if (!server_result.has_value()) {
    LOG_ERROR(logger_, "Failed to create WebSocket server: {}",
              foxglove::strerror(server_result.error()));
    return false;
  }
  socket_server_ = std::make_unique<WebSocketServer>(std::move(server_result.value()));

  auto make_channel = [](auto&& result) {
    return std::make_unique<std::decay_t<decltype(result.value())>>(std::move(result.value()));
  };

  loop_runtime_channel_ = make_channel(RawChannel::create("/simulation/runtime_secs", "json"));
  ego_car_channel_ = make_channel(SceneUpdateChannel::create("/markers/ego_car"));
  trajectory_channel_ = make_channel(SceneUpdateChannel::create("/markers/trajectory"));
  sampled_channel_ = make_channel(SceneUpdateChannel::create("/markers/sampled_trajectories"));
  reference_line_channel_ = make_channel(SceneUpdateChannel::create("/markers/reference_line"));
  obstacle_list_channel_ = make_channel(SceneUpdateChannel::create("/markers/obstacles"));
  obstacle_prediction_channel_ =
      make_channel(SceneUpdateChannel::create("/markers/obstacle_predictions"));
  transform_channel_ = make_channel(FrameTransformChannel::create("/transform/map_to_baselink"));

  auto planning_info_descriptor = protos::planning::PlanningInfo::descriptor();
  auto planning_info_schema =
      build_protobuf_schema(planning_info_descriptor, planning_info_schema_buffer_);
  planning_info_channel_ = make_channel(
      RawChannel::create("/planning_info", "protobuf", std::move(planning_info_schema)));

  auto config_descriptor = protos::config::SimulationConfig::descriptor();
  auto config_schema = build_protobuf_schema(config_descriptor, simulation_config_schema_buffer_);
  simulation_config_channel_ =
      make_channel(RawChannel::create("/simulation_config", "protobuf", std::move(config_schema)));

  return true;
}

foxglove::Schema Visualizer::build_protobuf_schema(
    const google::protobuf::Descriptor* message_descriptor,
    std::vector<std::uint8_t>& schema_buffer) const {
  foxglove::Schema schema;
  schema.encoding = "protobuf";
  schema.name = message_descriptor->full_name();
  google::protobuf::FileDescriptorSet file_descriptor_set;
  const google::protobuf::FileDescriptor* file_descriptor = message_descriptor->file();
  file_descriptor->CopyTo(file_descriptor_set.add_file());
  std::string serialized_descriptor = file_descriptor_set.SerializeAsString();
  schema_buffer.assign(serialized_descriptor.begin(), serialized_descriptor.end());
  schema.data = reinterpret_cast<const std::byte*>(schema_buffer.data());
  schema.data_len = schema_buffer.size();
  return schema;
}

// ============================================================================
// Public logging methods
// ============================================================================

void Visualizer::log_ego_state(const StateInfo& ego_state) {
  if (!running_) return;
  auto ego_car_scene_update = get_ego_scene_update(ego_state);
  ego_car_channel_->log(ego_car_scene_update);

  const auto ego_pose = ego_state.to_pose();
  foxglove::schemas::FrameTransform transform;
  transform.timestamp = TimeUtil::NowTimestamp();
  transform.parent_frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.translation = ego_pose.position;
  transform.rotation = ego_pose.orientation;
  transform_channel_->log(transform);
}

void Visualizer::log_transform(const foxglove::schemas::FrameTransform& transform) {
  if (!running_) return;
  transform_channel_->log(transform);
}

void Visualizer::log_loop_runtime(double elapsed_seconds) {
  if (!running_) return;
  std::string elapsed_msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
  loop_runtime_channel_->log(reinterpret_cast<const std::byte*>(elapsed_msg.data()),
                             elapsed_msg.size());
}

void Visualizer::log_planning_info(const protos::planning::PlanningInfo& planning_info) {
  if (!running_) return;
  std::string debug_info_data = planning_info.SerializeAsString();
  planning_info_channel_->log(reinterpret_cast<const std::byte*>(debug_info_data.data()),
                              debug_info_data.size());
  auto traj_scene_update = get_trajectory_scene_update(planning_info);
  trajectory_channel_->log(traj_scene_update);
  auto sampled_scene_update = get_sampled_scene_update(planning_info);
  sampled_channel_->log(sampled_scene_update);
}

void Visualizer::log_trajectory(const protos::planning::PlanningInfo& planning_info) {
  if (!running_) return;
  auto traj_scene_update = get_trajectory_scene_update(planning_info);
  trajectory_channel_->log(traj_scene_update);
}

void Visualizer::log_sampled_trajectories(const protos::planning::PlanningInfo& planning_info) {
  if (!running_) return;
  auto sampled_scene_update = get_sampled_scene_update(planning_info);
  sampled_channel_->log(sampled_scene_update);
}

void Visualizer::log_reference_line(const std::shared_ptr<ReferenceLine>& reference_line) {
  if (!running_) return;
  auto ref_line_scene_update = get_reference_line_scene_update(reference_line);
  reference_line_channel_->log(ref_line_scene_update);
}

void Visualizer::log_obstacles(const std::shared_ptr<common::ObstacleList>& obstacle_list,
                               const StateInfo& current_ego_state, double perception_range_m) {
  if (!running_) return;
  auto obstacles_scene_update = get_obstacle_list_scene_update(
      obstacle_list, current_ego_state, perception_range_m);
  obstacle_list_channel_->log(obstacles_scene_update);
}

void Visualizer::log_obstacle_predictions(
    const std::shared_ptr<common::ObstacleList>& obstacle_list,
    const StateInfo& current_ego_state, double perception_range_m) {
  if (!running_) return;
  auto prediction_scene_update = get_prediction_scene_update(
      obstacle_list, current_ego_state, perception_range_m);
  obstacle_prediction_channel_->log(prediction_scene_update);
}

void Visualizer::log_simulation_config(const protos::config::SimulationConfig& sim_config) {
  if (!running_) return;
  std::string config_data = sim_config.SerializeAsString();
  simulation_config_channel_->log(reinterpret_cast<const std::byte*>(config_data.data()),
                                  config_data.size());
}

// ============================================================================
// Scene update generation methods
// ============================================================================

foxglove::schemas::SceneUpdate Visualizer::get_ego_scene_update(const StateInfo& ego_state) const {
  ModelPrimitive ego_model;
  ego_model.url =
      "https://raw.githubusercontent.com/PuYuuu/mppi-in-autonomous-driving/master/assets/lexus.glb";
  ego_model.pose = construct_pose();
  ego_model.scale = {1, 1, 1};
  ego_model.media_type = "model/gltf-binary";
  ego_model.override_color = false;

  SceneEntity ego_car_entity;
  ego_car_entity.id = "ego_car";
  ego_car_entity.frame_id = "base_link";
  ego_car_entity.models.emplace_back(ego_model);
  ego_car_entity.timestamp = TimeUtil::NowTimestamp();
  ego_car_entity.lifetime = Duration{0, 200000000};

  SceneUpdate ego_car_scene_update;
  ego_car_scene_update.entities.emplace_back(ego_car_entity);

  return ego_car_scene_update;
}

foxglove::schemas::SceneUpdate Visualizer::get_trajectory_scene_update(
    const protos::planning::PlanningInfo& planning_info) const {
  SceneUpdate traj_scene_update;
  SceneEntity traj_entity;
  traj_entity.id = "trajectory";
  traj_entity.frame_id = "map";
  traj_entity.timestamp = TimeUtil::NowTimestamp();
  traj_entity.lifetime = Duration{0, 200000000};

  Color traj_color{0.0, 0.4, 0.75, 0.9};  // #0066BFE5
  std::vector<Point3> traj_points;
  const auto& optimal_trajectory = planning_info.optimal_trajectory();
  for (int idx = 2; idx < planning_info.optimal_trajectory_size(); ++idx) {
    const float x = optimal_trajectory.at(idx).pos_x();
    const float y = optimal_trajectory.at(idx).pos_y();
    Point3 point{x, y, 0.0};
    traj_points.emplace_back(point);
  }
  traj_entity.triangles.emplace_back(create_line_mesh(traj_points, 1.4, traj_color, true));
  traj_scene_update.entities.emplace_back(traj_entity);
  return traj_scene_update;
}

foxglove::schemas::SceneUpdate Visualizer::get_sampled_scene_update(
    const protos::planning::PlanningInfo& planning_info) const {
  SceneUpdate traj_scene_update;
  SceneEntity traj_entity;
  traj_entity.id = "sampled_trajectory";
  traj_entity.frame_id = "map";
  traj_entity.timestamp = TimeUtil::NowTimestamp();
  traj_entity.lifetime = Duration{0, 200000000};

  auto traj_pose = construct_pose();
  Color traj_color{0.0, 0.8, 0.8, 0.9};  // #00CCCCE6

  const auto& sampled_trajectory = planning_info.sample_xs();
  for (int seq_idx = 0; seq_idx < planning_info.sample_xs_size(); ++seq_idx) {
    LinePrimitive sampled_line;
    sampled_line.type = LinePrimitive::LineType::LINE_STRIP;
    sampled_line.pose = traj_pose;
    sampled_line.thickness = 0.2;
    sampled_line.scale_invariant = false;
    sampled_line.color = traj_color;
    const auto& trajectory = sampled_trajectory.at(seq_idx).trajectory();
    for (int idx = 0; idx < trajectory.size(); ++idx) {
      const float x = trajectory.at(idx).pos_x();
      const float y = trajectory.at(idx).pos_y();
      Point3 point{x, y, 0.0};
      sampled_line.points.emplace_back(point);
    }
    traj_entity.lines.emplace_back(sampled_line);
  }
  traj_scene_update.entities.emplace_back(traj_entity);
  return traj_scene_update;
}

foxglove::schemas::SceneUpdate Visualizer::get_reference_line_scene_update(
    const std::shared_ptr<ReferenceLine>& reference_line) {
  static SceneUpdate ref_line_scene_update;
  if (reference_line_initialized_.load()) {
    return ref_line_scene_update;
  }

  SceneEntity ref_line_entity;
  ref_line_entity.id = "reference_line";
  ref_line_entity.frame_id = "map";
  ref_line_entity.timestamp = TimeUtil::NowTimestamp();
  ref_line_entity.lifetime = Duration{0, 0};

  Color ref_line_color{0.36, 0.83, 0.66, 0.9};  // #5CD4A8E6
  Color road_edge_color{0.9, 0.2, 0.2, 0.9};    // #E63333E6
  LinePrimitive ref_line_primitive;
  ref_line_primitive.type = LinePrimitive::LineType::LINE_STRIP;
  ref_line_primitive.pose = construct_pose();
  ref_line_primitive.thickness = 0.6;
  ref_line_primitive.scale_invariant = false;
  ref_line_primitive.color = ref_line_color;
  LinePrimitive ref_left_edge_primitive = ref_line_primitive;
  ref_left_edge_primitive.color = road_edge_color;
  LinePrimitive ref_right_edge_primitive = ref_left_edge_primitive;

  const double sphere_radius = 0.6 * 2.0;
  Vector3 sphere_size{sphere_radius, sphere_radius, sphere_radius};
  if (reference_line) {
    // Check if road edges are available
    const auto& left_edge = reference_line->get_left_road_edge();
    const auto& right_edge = reference_line->get_right_road_edge();
    bool has_road_edges = !left_edge.empty() && !right_edge.empty();
    
    for (size_t idx = 0; idx < reference_line->size(); idx += 5) {
      double px = reference_line->at(idx).x();
      double py = reference_line->at(idx).y();
      double pz = reference_line->at(idx).z();
      
      // Default road edge distances if not available
      double left_edge_dist = has_road_edges ? left_edge[idx] : 3.5;
      double right_edge_dist = has_road_edges ? right_edge[idx] : 3.5;

      Point3 point{px, py, 0.0};
      ref_line_primitive.points.emplace_back(point);
      if (idx % 50 == 0) {
        SpherePrimitive ref_point_sphere;
        ref_point_sphere.pose = construct_pose();
        ref_point_sphere.pose->position->x = point.x;
        ref_point_sphere.pose->position->y = point.y;
        ref_point_sphere.size = sphere_size;
        ref_point_sphere.color = ref_line_color;
        ref_line_entity.spheres.emplace_back(ref_point_sphere);
      }
      Point3 left_edge_point{px - left_edge_dist * std::sin(pz), py + left_edge_dist * std::cos(pz),
                             0.0};
      ref_left_edge_primitive.points.emplace_back(left_edge_point);
      Point3 right_edge_point{px + right_edge_dist * std::sin(pz),
                              py - right_edge_dist * std::cos(pz), 0.0};
      ref_right_edge_primitive.points.emplace_back(right_edge_point);
    }
    ref_line_entity.lines.emplace_back(ref_line_primitive);
    ref_line_entity.lines.emplace_back(ref_left_edge_primitive);
    ref_line_entity.lines.emplace_back(ref_right_edge_primitive);
    ref_line_scene_update.entities.emplace_back(ref_line_entity);
  }
  reference_line_initialized_.store(true);

  return ref_line_scene_update;
}

foxglove::schemas::SceneUpdate Visualizer::get_obstacle_list_scene_update(
    const std::shared_ptr<common::ObstacleList>& obstacle_list,
    const StateInfo& current_ego_state, double perception_range_m) {
  SceneUpdate obstacles_scene_update;

  if (!obstacle_list) {
    return obstacles_scene_update;
  }

  for (const auto& obstacle : obstacle_list->obstacles()) {
    const double obstacle_x = obstacle.x();
    const double obstacle_y = obstacle.y();
    const bool is_static = obstacle.is_static();
    const std::string obstacle_id = "obstacle_" + obstacle.id() + (is_static ? "/static" : "/dynamic");
    
    double distance_to_ego =
        std::hypot(obstacle_x - current_ego_state.x, obstacle_y - current_ego_state.y);
    if (distance_to_ego > perception_range_m) {
      continue;
    }

    const double obstacle_length = obstacle.length();
    const double obstacle_width = obstacle.width();
    const double obstacle_heading = obstacle.heading();

    SceneEntity obstacle_entity;
    obstacle_entity.id = obstacle_id;
    obstacle_entity.frame_id = "map";
    obstacle_entity.timestamp = TimeUtil::NowTimestamp();
    obstacle_entity.lifetime = Duration{0, 200000000};

    CubePrimitive cube_marker;
    std::tie(cube_marker.size, cube_marker.color) =
        get_obstacle_size_and_color(static_cast<int>(obstacle.type()), obstacle_length, obstacle_width);
    double half_height = cube_marker.size->z / 2.0;
    Pose obstacle_pose;
    obstacle_pose.position = Vector3{obstacle_x, obstacle_y, half_height};
    obstacle_pose.orientation = yaw_to_quaternion(obstacle_heading);
    cube_marker.pose = obstacle_pose;
    obstacle_entity.cubes.emplace_back(cube_marker);

    TextPrimitive id_text;
    id_text.text = obstacle.id();
    id_text.pose = cube_marker.pose;
    id_text.color = Color{1.0, 1.0, 1.0, 1.0};
    id_text.font_size = obstacle_width;
    obstacle_entity.texts.emplace_back(id_text);

    obstacles_scene_update.entities.emplace_back(obstacle_entity);
  }

  return obstacles_scene_update;
}

foxglove::schemas::SceneUpdate Visualizer::get_prediction_scene_update(
    const std::shared_ptr<common::ObstacleList>& obstacle_list,
    const StateInfo& current_ego_state, double perception_range_m) {
  SceneUpdate prediction_scene_update;

  if (!obstacle_list) {
    return prediction_scene_update;
  }

  for (const auto& obstacle : obstacle_list->obstacles()) {
    const double obstacle_x = obstacle.x();
    const double obstacle_y = obstacle.y();
    const bool is_static = obstacle.is_static();
    
    double distance_to_ego =
        std::hypot(obstacle_x - current_ego_state.x, obstacle_y - current_ego_state.y);
    if (distance_to_ego > perception_range_m || is_static) {
      continue;
    }

    const auto& predicted_trajectory = obstacle.prediction();
    if (predicted_trajectory.empty()) {
      continue;
    }

    auto [dont_use_obs_size, obs_color] =
        get_obstacle_size_and_color(static_cast<int>(obstacle.type()), 0, 0);
    
    SceneEntity prediction_entity;
    prediction_entity.id = "prediction_" + obstacle.id();
    prediction_entity.frame_id = "map";
    prediction_entity.timestamp = TimeUtil::NowTimestamp();
    prediction_entity.lifetime = Duration{0, 200000000};
    
    LinePrimitive prediction_line;
    prediction_line.type = LinePrimitive::LineType::LINE_STRIP;
    prediction_line.pose = construct_pose();
    prediction_line.thickness = 0.45;
    prediction_line.scale_invariant = false;
    prediction_line.color = obs_color;
    
    for (size_t point_idx = 0; point_idx < predicted_trajectory.size(); point_idx += 4) {
      const auto& point = predicted_trajectory[point_idx];
      prediction_line.points.emplace_back(Point3{point.x, point.y, 0.0});
    }
    prediction_entity.lines.emplace_back(prediction_line);
    prediction_scene_update.entities.emplace_back(prediction_entity);
  }

  return prediction_scene_update;
}