/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-24
 * @FilePath: /mppi-in-autonomous-driving/modules/planning_node_standalone.cpp
 * Standalone planning node with lightweight simulator and Foxglove visualization
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "modules/planner/stochastic_optimizer.cuh"
#include "modules/simple_simulator/simple_simulator.hpp"
#include "modules/visualizer/visualizer.hpp"

#include <csignal>
#include <getopt.h>
#include <thread>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
  int opt;
  const char* optstring = "c:";
  std::string config_file_path;

  while ((opt = getopt(argc, argv, optstring)) != -1) {
    switch (opt) {
      case 'c':
        config_file_path = optarg;
        break;
      default:
        spdlog::error("Usage: {} -c <config_file>", argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  if (config_file_path.empty()) {
    spdlog::error("Usage: {} -c <config_file>", argv[0]);
    exit(EXIT_FAILURE);
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file_path);
  } catch (const YAML::Exception& e) {
    spdlog::error("Error parsing YAML file: {}", e.what());
    return 1;
  }

  spdlog::info("Standalone planning node starting...");
  spdlog::info("Config file: {}", config_file_path);

  // Initialize visualizer
  auto visualizer = std::make_shared<Visualizer>(config);
  if (!visualizer->start()) {
    spdlog::error("Failed to start visualizer");
    return 1;
  }

  // Initialize lightweight simulator
  auto simulator = std::make_unique<simple_simulator::SimpleSimulator>(config);
  simulator->start();

  std::atomic_bool done = false;
  static std::function<void()> sigint_handler = [&] { done = true; };
  std::signal(SIGINT, [](int) {
    if (sigint_handler) {
      sigint_handler();
      spdlog::info("SIGINT received, shutting down...");
    }
  });

  // Lambda to run closed-loop planning with visualization
  auto run_planning = [&](auto optimizer_ptr) {
    spdlog::info("Planner initialized with {} rollouts",
                 config["planning"]["mppi_params"]["num_samples"].as<int>());

    // Log reference line once at startup
    visualizer->log_reference_line(simulator->get_reference_line());

    // Planning loop rate (default 10 Hz)
    double planning_dt = 0.1;
    if (config["planning"]["dt"]) {
      planning_dt = config["planning"]["dt"].as<double>();
    }
    
    double perception_range = 100.0;
    if (config["simulation"]["perception_range"]) {
      perception_range = config["simulation"]["perception_range"].as<double>();
    }
    
    int frame_count = 0;
    auto next_tick = std::chrono::steady_clock::now();

    while (!done) {
      TicToc frame_timer;

      // ============================================================
      // 1. Get current state from simulator
      // ============================================================
      StateInfo ego_state = simulator->get_ego_state();

      // ============================================================
      // 2. Get reference line
      // ============================================================
      std::shared_ptr<ReferenceLine> reference_line = simulator->get_reference_line();

      // ============================================================
      // 3. Get obstacle list with predictions
      // ============================================================
      std::shared_ptr<common::ObstacleList> obstacle_list = simulator->get_obstacle_list();

      // ============================================================
      // 4. Core planning call - single frame
      // ============================================================
      ControlInput control = optimizer_ptr->plan_once(ego_state, reference_line, obstacle_list);

      // ============================================================
      // 5. Send control back to simulator (closed-loop)
      // ============================================================
      simulator->set_ego_control_input(control);

      // ============================================================
      // 6. Visualization
      // ============================================================
      visualizer->log_ego_state(ego_state);
      visualizer->log_obstacles(obstacle_list, ego_state, perception_range);
      visualizer->log_obstacle_predictions(obstacle_list, ego_state, perception_range);
      visualizer->log_loop_runtime(frame_timer.toc());

      // Logging
      spdlog::info("Frame {}: ego=({:.2f}, {:.2f}), v={:.2f} m/s, "
                   "accel={:.3f} m/s^2, steer={:.4f} rad, time={:.2f} ms",
                   frame_count, ego_state.x, ego_state.y, ego_state.velocity,
                   control.accel, control.steer, frame_timer.toc() * 1000.0);

      frame_count++;

      // Control loop rate
      next_tick += std::chrono::microseconds(static_cast<int64_t>(planning_dt * 1e6));
      std::this_thread::sleep_until(next_tick);
    }

    spdlog::info("Planning node stopped. Total frames: {}", frame_count);
  };

  // Select optimizer based on number of rollouts
  int num_rollouts = config["planning"]["mppi_params"]["num_samples"].as<int>(8192);
  switch (num_rollouts) {
    case 1024: {
      auto optimizer = std::make_unique<StochasticOptimizer<1024>>(config, visualizer);
      run_planning(std::move(optimizer));
      break;
    }
    case 2048: {
      auto optimizer = std::make_unique<StochasticOptimizer<2048>>(config, visualizer);
      run_planning(std::move(optimizer));
      break;
    }
    case 4096: {
      auto optimizer = std::make_unique<StochasticOptimizer<4096>>(config, visualizer);
      run_planning(std::move(optimizer));
      break;
    }
    case 8192: {
      auto optimizer = std::make_unique<StochasticOptimizer<8192>>(config, visualizer);
      run_planning(std::move(optimizer));
      break;
    }
    case 16384: {
      auto optimizer = std::make_unique<StochasticOptimizer<16384>>(config, visualizer);
      run_planning(std::move(optimizer));
      break;
    }
    default:
      spdlog::error("Unsupported num_rollouts: {}. Supported: 1024, 2048, 4096, 8192, 16384",
                    num_rollouts);
      simulator->stop();
      visualizer->stop();
      return 1;
  }

  simulator->stop();
  visualizer->stop();
  return 0;
}
