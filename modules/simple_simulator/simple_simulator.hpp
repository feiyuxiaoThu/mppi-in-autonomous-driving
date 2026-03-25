/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-24
 * @Description: Lightweight simulator to replace CommonRoad backend
 */

#pragma once

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/reference_line.hpp"

#ifdef USE_VISUALIZER
#include "modules/visualizer/visualizer.hpp"
#endif

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

namespace simple_simulator {

struct ObstacleConfig {
  std::string id;
  double x{0.0};
  double y{0.0};
  double velocity{0.0};
  double heading{0.0};
  double width{1.8};
  double length{4.5};
  int type{0};  // common::ObstacleType as int
  bool is_static{false};
  std::string prediction_mode{"constant_velocity"};  // "constant_velocity", "stationary"
};

struct EgoInitialState {
  double x{0.0};
  double y{0.0};
  double velocity{0.0};
  double heading{0.0};
  double accel{0.0};
  double steer{0.0};
};

struct ReferenceLineConfig {
  std::vector<double> x_points;
  std::vector<double> y_points;
  double accuracy{0.5};
};

class SimpleSimulator {
public:
  SimpleSimulator() = delete;
  SimpleSimulator(const YAML::Node& config);
  ~SimpleSimulator();

  bool start();
  void stop();

  StateInfo get_ego_state() const;
  void set_ego_control_input(const ControlInput& input);
  std::shared_ptr<ReferenceLine> get_reference_line() const;
  std::shared_ptr<common::ObstacleList> get_obstacle_list();

  // Get simulation timestep
  size_t get_timestep() const { return sim_timestep_.load(); }

private:
  void simulation_loop();
  void update_ego_state(double dt);
  void update_obstacles(double dt);
  std::vector<PathPoint> predict_obstacle(const ObstacleConfig& obs, size_t horizon_steps);
  void load_config(const YAML::Node& config);

  // Configuration
  EgoInitialState ego_initial_;
  std::vector<ObstacleConfig> obstacle_configs_;
  ReferenceLineConfig ref_line_config_;
  double perception_range_m_{100.0};
  double simulation_dt_{0.02};  // 50Hz simulation loop
  double wheelbase_{3.0};

  // State
  StateInfo ego_state_;
  std::vector<ObstacleConfig> current_obstacles_;
  std::shared_ptr<ReferenceLine> reference_line_{nullptr};

  // Threading
  std::thread sim_thread_;
  std::atomic<bool> running_{false};
  std::atomic<size_t> sim_timestep_{0};
  mutable std::shared_mutex ego_state_mutex_;
  mutable std::shared_mutex obstacles_mutex_;

  // Logger
  std::shared_ptr<spdlog::logger> logger_{nullptr};
};

}  // namespace simple_simulator
