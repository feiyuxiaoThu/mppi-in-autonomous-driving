/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-30
 * @FilePath: /mppi-in-autonomous-driving/modules/biased_mppi_demo.cpp
 * Demo node for Biased-MPPI with simulated E2E priors
 * Copyright (c) 2026 by puyu, All Rights Reserved.
 */

#include "modules/planner/stochastic_optimizer.cuh"
#include "modules/simple_simulator/simple_simulator.hpp"
#include "modules/visualizer/visualizer.hpp"

#include <csignal>
#include <thread>
#include <yaml-cpp/yaml.h>

/**
 * @brief Generate dummy E2E prior trajectories based on the reference line
 * Simulated 2 modes: One shifting left, one shifting right for obstacle avoidance
 */
std::vector<E2EPriorMode> GenerateDummyE2EPriors(
    const StateInfo& ego_state, const std::shared_ptr<ReferenceLine>& ref_line, int horizon, double dt) {
  
  std::vector<E2EPriorMode> priors;
  if (!ref_line || ref_line->size() == 0) return priors;

  // Find the closest point in the reference line to the ego state
  double min_dist = std::numeric_limits<double>::max();
  double cur_s = 0.0;
  
  for (size_t i = 0; i < ref_line->size(); ++i) {
    auto pt = ref_line->at(i);
    double dx = ego_state.x - pt.x();
    double dy = ego_state.y - pt.y();
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      cur_s = ref_line->longitude_[i];
    }
  }

  // Prior 1: Left bias (offset = +2.5m), Confidence = 0.7
  // Prior 2: Right bias (offset = -2.5m), Confidence = 0.3
  std::vector<std::pair<double, float>> offsets_with_conf = {{2.5, 0.7f}, {-2.5, 0.3f}};

  int mode_id = 0;
  for (const auto& pair : offsets_with_conf) {
    double offset = pair.first;
    float confidence = pair.second;
    
    std::vector<PathPoint> path;
    for (int i = 0; i < horizon + 2; ++i) {
      double target_s = cur_s + std::max(ego_state.velocity, 5.0) * i * dt;
      target_s = std::min(target_s, ref_line->length());

      Eigen::Vector3d ref_pt = ref_line->calc_position(target_s);
      double rx = ref_pt.x();
      double ry = ref_pt.y();
      double ryaw = ref_pt.z();
      
      // Calculate offset position along the normal of the reference line
      double x = rx - offset * std::sin(ryaw);
      double y = ry + offset * std::cos(ryaw);
      
      path.push_back({x, y, ryaw, std::max(ego_state.velocity, 5.0), i * dt});
    }
    
    E2EPriorMode mode;
    mode.trajectory = path;
    mode.confidence = confidence;
    mode.mode_id = ++mode_id;
    priors.push_back(mode);
  }

  return priors;
}

int main(int argc, char** argv) {
  std::string config_file_path = "config/standalone.yaml";
  if (argc > 1) config_file_path = argv[1];

  YAML::Node config = YAML::LoadFile(config_file_path);
  spdlog::info("Biased-MPPI Demo starting with config: {}", config_file_path);

  auto visualizer = std::make_shared<Visualizer>(config);
  visualizer->start();

  auto simulator = std::make_unique<simple_simulator::SimpleSimulator>(config);
  simulator->start();

  // Initialize Optimizer (using 4096 rollouts for good visualization)
  auto optimizer = std::make_unique<StochasticOptimizer<4096>>(config, visualizer);

  std::atomic_bool done = false;
  std::signal(SIGINT, [](int) { exit(0); });

  double planning_dt = 0.1;
  auto next_tick = std::chrono::steady_clock::now();

  while (!done) {
    TicToc frame_timer;
    StateInfo ego_state = simulator->get_ego_state();
    std::shared_ptr<ReferenceLine> reference_line = simulator->get_reference_line();
    std::shared_ptr<common::ObstacleList> obstacle_list = simulator->get_obstacle_list();

    // 1. Generate Simulated E2E Priors
    auto e2e_priors = GenerateDummyE2EPriors(ego_state, reference_line, kHorizonLength, planning_dt);

    // 2. Core planning call with Priors
    ControlInput control = optimizer->plan_once(ego_state, reference_line, obstacle_list, e2e_priors);

    // 3. Simulator & Visualizer update
    simulator->set_ego_control_input(control);
    visualizer->log_ego_state(ego_state);
    visualizer->log_reference_line(reference_line);
    visualizer->log_obstacles(obstacle_list, ego_state, 100.0);
    
    spdlog::info("Frame: v={:.2f}, priors_count={}, cost={:.2f}ms", 
                 ego_state.velocity, e2e_priors.size(), frame_timer.toc() * 1000.0);

    next_tick += std::chrono::microseconds(static_cast<int64_t>(planning_dt * 1e6));
    std::this_thread::sleep_until(next_tick);
  }

  return 0;
}
