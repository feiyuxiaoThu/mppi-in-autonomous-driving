/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:16:24
 * @LastEditTime: 2026-01-21 00:01:05
 * @FilePath: /mppi-in-autonomous-driving/modules/planner/stochastic_optimizer.cuh
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/protos/planning_info.pb.h"
#include "common/protos/config.pb.h"
#include "common/reference_line.hpp"
#include "mppi/controllers/MPPI/mppi_controller.cuh"
#include "mppi/feedback_controllers/DDP/ddp.cuh"
#include "trajectory_cost.cuh"
#include "vehicle_dynamics.cuh"

#ifdef USE_VISUALIZER
#include "modules/visualizer/visualizer.hpp"
#endif

#include <Eigen/Core>
#include <memory>

// Forward declaration for standalone builds without visualizer
class Visualizer;

template <int NUM_ROLLOUTS>
class StochasticOptimizer {
public:
  StochasticOptimizer() = delete;
  StochasticOptimizer(const YAML::Node& config, std::shared_ptr<Visualizer> visualizer = nullptr);
  ~StochasticOptimizer();

  ControlInput plan_once(const StateInfo& _current_state,
                         const std::shared_ptr<ReferenceLine>& reference_line,
                         const std::shared_ptr<common::ObstacleList>& obstacle_list);
  Eigen::MatrixXf get_optimized_trajectory() const;
  protos::planning::PlanningInfo get_debug_result(const StateInfo& current_state) const;
  void set_parameters_to_proto(protos::config::SimulationConfig* sim_config_ptr);

private:
  using SAMPLER_T =
    mppi::sampling_distributions::GaussianDistribution<VehicleDynamics::DYN_PARAMS_T>;

  void convert_parameters_to_proto(const TrajectoryCostParams& cost_params,
                                   const VehicleDynamics& vehicle_dynamics,
                                   const SAMPLER_T::SAMPLING_PARAMS_T& sampler_params);

  double delta_time_ = 0.1;
  VehicleDynamics* dynamics_ = nullptr;
  TrajectoryCost* trajectory_cost_ = nullptr;
  SAMPLER_T* sampler_ = nullptr;
  DDPFeedback<VehicleDynamics, kHorizonLength>* ddp_feedback_ = nullptr;
  VanillaMPPIController<VehicleDynamics, TrajectoryCost,
                        DDPFeedback<VehicleDynamics, kHorizonLength>, kHorizonLength, NUM_ROLLOUTS>*
      mppi_controller_ = nullptr;

  float target_accel_ = 0.0;
  float target_steer_ = 0.0;
  float cruise_velocity_ = 15.0;
  float cost_time_ms_ = 0.0;

  protos::config::VehicleInfo vehicle_info_;
  protos::config::PlanningParameters planning_parameters_;

  std::shared_ptr<spdlog::logger> logger_ = nullptr;
  std::shared_ptr<Visualizer> visualizer_{nullptr};
};
