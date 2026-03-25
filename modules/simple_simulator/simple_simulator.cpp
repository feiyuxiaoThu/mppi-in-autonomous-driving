/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-24
 * @Description: Lightweight simulator to replace CommonRoad backend
 */

#include "modules/simple_simulator/simple_simulator.hpp"

#include <cmath>

namespace simple_simulator {

SimpleSimulator::SimpleSimulator(const YAML::Node& config)
    : logger_(spdlog::stdout_color_mt("simple_simulator")) {
  load_config(config);
  
  // Initialize ego state
  ego_state_.x = ego_initial_.x;
  ego_state_.y = ego_initial_.y;
  ego_state_.velocity = ego_initial_.velocity;
  ego_state_.heading = ego_initial_.heading;
  ego_state_.accel = ego_initial_.accel;
  ego_state_.steer = ego_initial_.steer;
  
  // Initialize obstacles
  current_obstacles_ = obstacle_configs_;
  
  // Initialize reference line
  if (!ref_line_config_.x_points.empty() && !ref_line_config_.y_points.empty()) {
    reference_line_ = std::make_shared<ReferenceLine>(
        ref_line_config_.x_points, ref_line_config_.y_points);
    spdlog::info("Reference line created with {} points, accuracy = {}",
                 ref_line_config_.x_points.size(), ref_line_config_.accuracy);
  }
  
  spdlog::info("SimpleSimulator initialized: ego=({:.2f}, {:.2f}), {} obstacles",
               ego_state_.x, ego_state_.y, current_obstacles_.size());
}

SimpleSimulator::~SimpleSimulator() {
  stop();
}

void SimpleSimulator::load_config(const YAML::Node& config) {
  // Load simulation parameters
  if (config["simulation"]) {
    auto sim = config["simulation"];
    if (sim["dt"]) simulation_dt_ = sim["dt"].as<double>();
    if (sim["wheelbase"]) wheelbase_ = sim["wheelbase"].as<double>();
    if (sim["perception_range"]) perception_range_m_ = sim["perception_range"].as<double>();
  }
  
  // Load ego initial state
  if (config["ego"]) {
    auto ego = config["ego"];
    ego_initial_.x = ego["x"].as<double>(0.0);
    ego_initial_.y = ego["y"].as<double>(0.0);
    ego_initial_.velocity = ego["velocity"].as<double>(0.0);
    ego_initial_.heading = ego["heading"].as<double>(0.0);
    ego_initial_.accel = ego["accel"].as<double>(0.0);
    ego_initial_.steer = ego["steer"].as<double>(0.0);
  }
  
  // Load reference line
  if (config["reference_line"]) {
    auto ref = config["reference_line"];
    ref_line_config_.x_points = ref["x"].as<std::vector<double>>(std::vector<double>());
    ref_line_config_.y_points = ref["y"].as<std::vector<double>>(std::vector<double>());
    ref_line_config_.accuracy = ref["accuracy"].as<double>(0.5);
  }
  
  // Load obstacles
  if (config["obstacles"]) {
    for (const auto& obs : config["obstacles"]) {
      ObstacleConfig obs_cfg;
      obs_cfg.id = obs["id"].as<std::string>("unknown");
      obs_cfg.x = obs["x"].as<double>(0.0);
      obs_cfg.y = obs["y"].as<double>(0.0);
      obs_cfg.velocity = obs["velocity"].as<double>(0.0);
      obs_cfg.heading = obs["heading"].as<double>(0.0);
      obs_cfg.width = obs["width"].as<double>(1.8);
      obs_cfg.length = obs["length"].as<double>(4.5);
      obs_cfg.type = obs["type"].as<int>(0);
      obs_cfg.is_static = obs["is_static"].as<bool>(false);
      obs_cfg.prediction_mode = obs["prediction_mode"].as<std::string>("constant_velocity");
      obstacle_configs_.push_back(obs_cfg);
    }
  }
}

bool SimpleSimulator::start() {
  if (running_.load()) {
    logger_->warn("SimpleSimulator already running");
    return false;
  }
  
  running_.store(true);
  sim_thread_ = std::thread(&SimpleSimulator::simulation_loop, this);
  logger_->info("SimpleSimulator started at {} Hz", 1.0 / simulation_dt_);
  return true;
}

void SimpleSimulator::stop() {
  running_.store(false);
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }
  logger_->info("SimpleSimulator stopped at timestep {}", sim_timestep_.load());
}

void SimpleSimulator::simulation_loop() {
  auto next_tick = std::chrono::steady_clock::now();
  
  while (running_.load()) {
    TicToc loop_timer;
    
    // Update ego state (using last control input)
    update_ego_state(simulation_dt_);
    
    // Update obstacles
    update_obstacles(simulation_dt_);
    
    // Increment timestep
    sim_timestep_.fetch_add(1);
    
    // Control loop rate
    next_tick += std::chrono::microseconds(static_cast<int64_t>(simulation_dt_ * 1e6));
    std::this_thread::sleep_until(next_tick);
  }
}

void SimpleSimulator::update_ego_state(double dt) {
  std::unique_lock lock(ego_state_mutex_);
  
  // Bicycle model state prediction
  // dx/dt = v * cos(heading)
  // dy/dt = v * sin(heading)
  // dheading/dt = v * tan(steer) / wheelbase
  // dvelocity/dt = accel
  
  ego_state_.x += ego_state_.velocity * std::cos(ego_state_.heading) * dt;
  ego_state_.y += ego_state_.velocity * std::sin(ego_state_.heading) * dt;
  ego_state_.heading += ego_state_.velocity * std::tan(ego_state_.steer) / wheelbase_ * dt;
  ego_state_.velocity += ego_state_.accel * dt;
  
  // Clamp velocity to non-negative
  ego_state_.velocity = std::max(0.0, ego_state_.velocity);
  
  // Normalize heading to [-pi, pi]
  while (ego_state_.heading > M_PI) ego_state_.heading -= 2 * M_PI;
  while (ego_state_.heading < -M_PI) ego_state_.heading += 2 * M_PI;
}

void SimpleSimulator::update_obstacles(double dt) {
  std::unique_lock lock(obstacles_mutex_);
  
  for (auto& obs : current_obstacles_) {
    if (obs.is_static) {
      continue;  // Static obstacles don't move
    }
    
    if (obs.prediction_mode == "constant_velocity") {
      // Constant velocity model
      obs.x += obs.velocity * std::cos(obs.heading) * dt;
      obs.y += obs.velocity * std::sin(obs.heading) * dt;
    }
    // Other prediction modes can be added here
  }
}

StateInfo SimpleSimulator::get_ego_state() const {
  std::shared_lock lock(ego_state_mutex_);
  return ego_state_;
}

void SimpleSimulator::set_ego_control_input(const ControlInput& input) {
  std::unique_lock lock(ego_state_mutex_);
  ego_state_.accel = input.accel;
  ego_state_.steer = input.steer;
}

std::shared_ptr<ReferenceLine> SimpleSimulator::get_reference_line() const {
  return reference_line_;
}

std::shared_ptr<common::ObstacleList> SimpleSimulator::get_obstacle_list() {
  auto obstacle_list = std::make_shared<common::ObstacleList>();
  
  std::shared_lock lock(obstacles_mutex_);
  
  // Get ego state for range filtering
  StateInfo ego = get_ego_state();
  
  for (const auto& obs : current_obstacles_) {
    // Filter by perception range
    double dx = obs.x - ego.x;
    double dy = obs.y - ego.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist <= perception_range_m_) {
      // Create obstacle with prediction
      std::vector<PathPoint> prediction = predict_obstacle(obs, 30);  // 30 steps prediction
      
      common::Obstacle obstacle(
          obs.id,
          obs.x,
          obs.y,
          obs.heading,
          obs.width,
          obs.length,
          obs.is_static,
          static_cast<common::ObstacleType>(obs.type)
      );
      obstacle.set_prediction(prediction);
      
      obstacle_list->append(obstacle);
    }
  }
  
  return obstacle_list;
}

std::vector<PathPoint> SimpleSimulator::predict_obstacle(const ObstacleConfig& obs, size_t horizon_steps) {
  std::vector<PathPoint> prediction;
  prediction.reserve(horizon_steps);
  
  if (obs.is_static) {
    // Static obstacle: same position for all time steps
    for (size_t i = 0; i < horizon_steps; ++i) {
      PathPoint pt;
      pt.x = obs.x;
      pt.y = obs.y;
      pt.yaw = obs.heading;
      pt.v = 0.0;
      pt.t = i * simulation_dt_;
      prediction.push_back(pt);
    }
  } else if (obs.prediction_mode == "constant_velocity") {
    // Constant velocity prediction
    double x = obs.x;
    double y = obs.y;
    double yaw = obs.heading;
    double dt = simulation_dt_;
    
    for (size_t i = 0; i < horizon_steps; ++i) {
      PathPoint pt;
      pt.x = x;
      pt.y = y;
      pt.yaw = yaw;
      pt.v = obs.velocity;
      pt.t = i * dt;
      prediction.push_back(pt);
      x += obs.velocity * std::cos(yaw) * dt;
      y += obs.velocity * std::sin(yaw) * dt;
    }
  } else {
    // Default: stationary
    for (size_t i = 0; i < horizon_steps; ++i) {
      PathPoint pt;
      pt.x = obs.x;
      pt.y = obs.y;
      pt.yaw = obs.heading;
      pt.v = 0.0;
      pt.t = i * simulation_dt_;
      prediction.push_back(pt);
    }
  }
  
  return prediction;
}

}  // namespace simple_simulator
