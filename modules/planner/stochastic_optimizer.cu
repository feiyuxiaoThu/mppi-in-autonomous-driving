/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:39:47
 * @LastEditTime: 2026-01-24 22:49:04
 * @FilePath: /mppi-in-autonomous-driving/modules/planner/stochastic_optimizer.cu
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "stochastic_optimizer.cuh"

#define ENABLE_COST_VERIFICATION 0

template <int NUM_ROLLOUTS>
StochasticOptimizer<NUM_ROLLOUTS>::StochasticOptimizer(
    const YAML::Node& config, std::shared_ptr<Visualizer> visualizer /* = nullptr */)
    : visualizer_(visualizer) {
  auto planning_config = config["planning"];
  auto constraint_limits = planning_config["constraint_limits"];
  auto cost_weights = planning_config["cost_weights"];
  auto mppi_config = planning_config["mppi_params"];
  auto visualization_config = config["visualization"];
  auto vehicle_info = config["vehicle_info"];

  int visualize_sampled_num =
      visualization_config["sampled_trajectories_visualize_num"].as<int>(128);
  std::string visualize_sampled_method =
      visualization_config["sampled_trajectories_visualize_method"].as<std::string>("top_k");

  std::string planning_log_level = planning_config["log_level"].as<std::string>("info");
  logger_ = create_logger("planner_logger", planning_log_level);

  double wheel_base = vehicle_info["wheel_base_m"].as<float>(3.0f);
  dynamics_ = new VehicleDynamics(wheel_base);
  dynamics_->control_rngs_[0].x = constraint_limits["min_jerk_mps3"].as<float>(-1.5);
  dynamics_->control_rngs_[0].y = constraint_limits["max_jerk_mps3"].as<float>(1.5);
  dynamics_->control_rngs_[1].x = -constraint_limits["max_steer_rate_rps"].as<float>(0.07);
  dynamics_->control_rngs_[1].y = constraint_limits["max_steer_rate_rps"].as<float>(0.07);

  cruise_velocity_ = planning_config["desired_speed"].as<float>(16.0f);
  trajectory_cost_ = new TrajectoryCost;
  double jerk_weight = cost_weights["jerk_effort_weight"].as<float>(20.0f);
  double steer_rate_weight = cost_weights["steer_rate_effort_weight"].as<float>(100.0f);
  LOG_INFO(logger_, "Jerk weight: {}, Steer rate weight: {}", jerk_weight, steer_rate_weight);
  TrajectoryCostParams new_params(jerk_weight, steer_rate_weight);
  new_params.horizon_length = kHorizonLength;
  new_params.target_velocity = cruise_velocity_;
  new_params.position_coeff = cost_weights["position_weight"].as<float>(15.0f);
  new_params.velocity_coeff = cost_weights["velocity_weight"].as<float>(10.0f);
  new_params.angle_coeff = cost_weights["heading_weight"].as<float>(75.0f);
  new_params.accel_effort_coeff = cost_weights["accel_effort_weight"].as<float>(10.0f);
  new_params.steer_effort_coeff = cost_weights["steer_effort_weight"].as<float>(80.0f);
  new_params.max_accel = constraint_limits["max_accel_mps2"].as<float>(2.0f);
  new_params.min_accel = constraint_limits["min_accel_mps2"].as<float>(-4.0f);
  new_params.max_steer_angle = constraint_limits["max_steer_angle_rad"].as<float>(0.15f);
  new_params.min_steer_angle = -constraint_limits["max_steer_angle_rad"].as<float>(0.15f);
  new_params.control_cost_coeff[0] = cost_weights["jerk_effort_weight"].as<float>(20.0f);
  new_params.control_cost_coeff[1] = cost_weights["steer_rate_effort_weight"].as<float>(120.0f);
  new_params.longitudinal_safety_margin =
      constraint_limits["longitudinal_safety_margin_m"].as<float>(3.0f);
  new_params.lateral_safety_margin = constraint_limits["lateral_safety_margin_m"].as<float>(0.6f);
  new_params.wheelbase = wheel_base;
  new_params.vehicle_width = vehicle_info["vehicle_width_m"].as<float>(1.85f);
  new_params.vehicle_length = vehicle_info["vehicle_length_m"].as<float>(4.85f);
  new_params.axle_to_front_bumper = vehicle_info["axle_to_front_bumper_m"].as<float>(3.90f);
  new_params.axle_to_rear_bumper = vehicle_info["axle_to_rear_bumper_m"].as<float>(0.95f);
  trajectory_cost_->setParams(new_params);

  float delta_time_ = 0.1;
  int max_iter = 1;
  float lambda = mppi_config["lambda"].as<float>(2.5);
  float alpha = 0.0;

  auto sampler_params = SAMPLER_T::SAMPLING_PARAMS_T();
  sampler_params.std_dev[0] = mppi_config["noise_sigma"]["jerk_std"].as<float>(0.5);
  sampler_params.std_dev[1] = mppi_config["noise_sigma"]["steer_rate_std"].as<float>(0.02);
  sampler_ = new SAMPLER_T(sampler_params);

  ddp_feedback_ = new DDPFeedback<VehicleDynamics, kHorizonLength>(dynamics_, delta_time_);
  mppi_controller_ = new VanillaMPPIController<VehicleDynamics, TrajectoryCost,
                                               DDPFeedback<VehicleDynamics, kHorizonLength>,
                                               kHorizonLength, NUM_ROLLOUTS>(
      dynamics_, trajectory_cost_, ddp_feedback_, sampler_, delta_time_, max_iter, lambda, alpha);
  auto controller_params = mppi_controller_->getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  mppi_controller_->setParams(controller_params);

  if (visualize_sampled_method == "top_k") {
    mppi_controller_->setTopNSampledControlTrajectoriesHelper(visualize_sampled_num);
  } else {
    mppi_controller_->setPercentageSampledControlTrajectoriesHelper(
        static_cast<float>(visualize_sampled_num) / NUM_ROLLOUTS);
  }
  convert_parameters_to_proto(new_params, *dynamics_, sampler_params);
}

template <int NUM_ROLLOUTS>
StochasticOptimizer<NUM_ROLLOUTS>::~StochasticOptimizer() {
  delete dynamics_;
  delete trajectory_cost_;
  delete sampler_;
  delete ddp_feedback_;
  delete mppi_controller_;
}

template <int NUM_ROLLOUTS>
ControlInput StochasticOptimizer<NUM_ROLLOUTS>::plan_once(
    const StateInfo& _current_state, const std::shared_ptr<ReferenceLine>& reference_line,
    const std::shared_ptr<common::ObstacleList>& obstacle_list) {
  // Update waypoints in trajectory cost function
  if (reference_line) {
    trajectory_cost_->setWaypoints(reference_line);
    // Initialize matched index cache with current vehicle position
    trajectory_cost_->updateMatchedIndex(_current_state.x, _current_state.y);
  }

  // Update obstacles in trajectory cost function
  if (obstacle_list) {
    trajectory_cost_->setObstacles(obstacle_list);
    // Force synchronize params to GPU device
    trajectory_cost_->paramsToDevice();
    LOG_DEBUG(logger_, "Updated {} obstacles to GPU", obstacle_list->obstacles().size());
  }

  VehicleDynamics::state_array cur_state;
  VehicleDynamics::state_array next_state = dynamics_->getZeroState();
  VehicleDynamics::state_array xdot = dynamics_->getZeroState();
  VehicleDynamics::output_array output = VehicleDynamics::output_array::Zero();
  cur_state(0) = _current_state.x;
  cur_state(1) = _current_state.y;
  cur_state(2) = _current_state.velocity;
  cur_state(3) = _current_state.heading;
  cur_state(4) = _current_state.accel;
  cur_state(5) = _current_state.steer;

  TicToc solve_time_tic;
  mppi_controller_->computeControl(cur_state, 1);
  VehicleDynamics::control_array control =
      mppi_controller_->getControlSeq().block(0, 0, VehicleDynamics::CONTROL_DIM, 1);
  dynamics_->enforceConstraints(cur_state, control);
  dynamics_->step(cur_state, next_state, xdot, control, output, 0, delta_time_);
  mppi_controller_->slideControlSequence(1);
  cost_time_ms_ = solve_time_tic.toc() * 1000.0;
  LOG_DEBUG(logger_, "StochasticOptimizer plan_once cost time: {:.2f} ms", cost_time_ms_);

  target_accel_ = next_state(4);
  target_steer_ = next_state(5);

  return {target_accel_, target_steer_};
}

template <int NUM_ROLLOUTS>
Eigen::MatrixXf StochasticOptimizer<NUM_ROLLOUTS>::get_optimized_trajectory() const {
  return mppi_controller_->getTargetStateSeq();
}

template <int NUM_ROLLOUTS>
protos::planning::PlanningInfo StochasticOptimizer<NUM_ROLLOUTS>::get_debug_result(
    const StateInfo& current_state) const {
  TicToc get_debug_result_tic;
  protos::planning::PlanningInfo planning_info;
  planning_info.set_target_accel(target_accel_);
  planning_info.set_target_steer(target_steer_);
  planning_info.set_cruise_velocity(cruise_velocity_);
  planning_info.set_cost_time_ms(cost_time_ms_);
  planning_info.set_optimal_cost(mppi_controller_->getBaselineCost());

  // Set current state
  auto* state_proto = planning_info.mutable_current_state();
  state_proto->set_pos_x(current_state.x);
  state_proto->set_pos_y(current_state.y);
  state_proto->set_velocity(current_state.velocity);
  state_proto->set_heading(current_state.heading);
  state_proto->set_accel(current_state.accel);
  state_proto->set_steer(current_state.steer);

  // Set optimized trajectory
  Eigen::MatrixXf optimized_trajectory = mppi_controller_->getTargetStateSeq();
  for (int t = 0; t < optimized_trajectory.cols(); ++t) {
    auto* traj_point = planning_info.add_optimal_trajectory();
    traj_point->set_pos_x(optimized_trajectory(0, t));
    traj_point->set_pos_y(optimized_trajectory(1, t));
    traj_point->set_velocity(optimized_trajectory(2, t));
    traj_point->set_heading(optimized_trajectory(3, t));
    traj_point->set_accel(optimized_trajectory(4, t));
    traj_point->set_steer(optimized_trajectory(5, t));
  }
  // Set optimal control sequence
  Eigen::MatrixXf optimal_control_seq = mppi_controller_->getControlSeq();
  for (int t = 0; t < optimal_control_seq.cols(); ++t) {
    auto* control_proto = planning_info.add_optimal_control_seq();
    control_proto->set_jerk(optimal_control_seq(0, t));
    control_proto->set_steer_rate(optimal_control_seq(1, t));
  }

  // This step is a bit time-consuming and is only opened when debugging and visualizing samples.
  TicToc calculate_sample_tic;
  mppi_controller_->calculateSampledStateTrajectories();
  auto sampled_trajectories = mppi_controller_->getSampledOutputTrajectories();
  auto sampled_costs = mppi_controller_->getSampledCostTrajectories();
  LOG_DEBUG(logger_, "StochasticOptimizer calculate sampled trajectories cost time: {:.2f} ms",
            calculate_sample_tic.toc() * 1000.0);

#if ENABLE_COST_VERIFICATION
  // Verify optimal trajectory cost by recalculating on host
  LOG_INFO(logger_, "========== Cost Verification for Optimal Trajectory ==========");

  // Log obstacle information from cost function
  auto cost_params = trajectory_cost_->getParams();
  std::ostringstream oss;
  oss << "Obstacle count in cost params: " << cost_params.obstacle_list.count
      << ", Reference line count: " << cost_params.reference_line.count
      << ", Reference line ptr: " << (void*)cost_params.reference_line.waypoints;
  LOG_INFO(logger_, oss.str());

  Eigen::MatrixXf optimal_traj = mppi_controller_->getTargetStateSeq();
  float total_recalculated_cost = 0.0f;
  float total_reported_cost = mppi_controller_->getBaselineCost();

  // === GPU Verification: Copy trajectory to GPU and compute cost on device ===
  int num_steps = optimal_traj.cols();

  // Allocate GPU memory for states and costs
  float* states_gpu = nullptr;
  float* costs_gpu = nullptr;
  float* obstacle_costs_gpu = nullptr;
  cudaMalloc(&states_gpu, num_steps * 6 * sizeof(float));
  cudaMalloc(&costs_gpu, num_steps * sizeof(float));
  cudaMalloc(&obstacle_costs_gpu, num_steps * sizeof(float));

  // Copy trajectory data to GPU (transpose from column-major to row-major)
  std::vector<float> states_host(num_steps * 6);
  for (int t = 0; t < num_steps; ++t) {
    for (int i = 0; i < 6; ++i) {
      states_host[t * 6 + i] = optimal_traj(i, t);
    }
  }
  cudaMemcpy(states_gpu, states_host.data(), num_steps * 6 * sizeof(float), cudaMemcpyHostToDevice);

  // Initialize GPU cache: Ensure last_matched_idx is synced to GPU
  // The updateMatchedIndex was called in plan_once, so host cache is already correct
  // We just need to ensure it's synced to GPU before verification
  trajectory_cost_->paramsToDevice();

  auto cost_params_for_log = trajectory_cost_->getParams();
  LOG_INFO(logger_, "GPU cache initialized with matched_idx=" +
                        std::to_string(cost_params_for_log.reference_line.last_matched_idx));

  // Launch GPU verification kernel with single thread to maintain cache consistency
  // The last_matched_idx cache needs to be propagated across timesteps
  verifyCostOnGPUKernel<<<1, 1>>>(trajectory_cost_->cost_d_, states_gpu, costs_gpu,
                                  obstacle_costs_gpu, num_steps);
  cudaDeviceSynchronize();

  // Copy results back to host
  std::vector<float> costs_from_gpu(num_steps);
  std::vector<float> obs_costs_from_gpu(num_steps);
  cudaMemcpy(costs_from_gpu.data(), costs_gpu, num_steps * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(obs_costs_from_gpu.data(), obstacle_costs_gpu, num_steps * sizeof(float),
             cudaMemcpyDeviceToHost);

  float total_gpu_cost = 0.0f;
  for (int t = 0; t < num_steps; ++t) {
    total_gpu_cost += costs_from_gpu[t];
  }

  LOG_INFO(logger_, "GPU Cost Calculation: Total=" + std::to_string(total_gpu_cost));

  // Free GPU memory
  cudaFree(states_gpu);
  cudaFree(costs_gpu);
  cudaFree(obstacle_costs_gpu);
  // === End GPU Verification ===

  for (int t = 0; t < optimal_traj.cols(); ++t) {
    VehicleDynamics::output_array state = optimal_traj.col(t);
    float step_cost = trajectory_cost_->computeStateCost(state, t, nullptr);
    total_recalculated_cost += step_cost;

    // Log detailed cost breakdown every 10 steps or on first/last step
    if (t == 0 || t == optimal_traj.cols() - 1 || t % 10 == 0) {
      float obs_cost_host = trajectory_cost_->computeObstacleCost(state(0), state(1), state(3), t);
      float step_cost_gpu = costs_from_gpu[t];
      float obs_cost_gpu = obs_costs_from_gpu[t];

      // Compute detailed cost breakdown on host
      float x = state(0), y = state(1), vel = state(2), heading = state(3);
      float accel = state(4), steer = state(5);

      auto cost_params_host = trajectory_cost_->getParams();
      float pos_cost = cost_params_host.position_coeff * std::abs(0.0f);  // Simplified
      float vel_cost =
          cost_params_host.velocity_coeff * std::abs(vel - cost_params_host.target_velocity);
      float angle_cost = cost_params_host.angle_coeff * 0.0f;  // Simplified
      float accel_cost = cost_params_host.accel_effort_coeff * std::abs(accel);
      float steer_cost = cost_params_host.steer_effort_coeff * std::abs(steer);
      float other_cost = step_cost - obs_cost_host - vel_cost - accel_cost - steer_cost;

      std::ostringstream step_oss;
      step_oss << "Step[" << t << "] pos=(" << x << "," << y << ") vel=" << vel
               << " heading=" << heading << "\n"
               << "  Host: total=" << step_cost << " obs=" << obs_cost_host << " vel=" << vel_cost
               << " accel=" << accel_cost << " steer=" << steer_cost << " other=" << other_cost
               << "\n"
               << "  GPU:  total=" << step_cost_gpu << " obs=" << obs_cost_gpu
               << " (diff=" << std::abs(step_cost - step_cost_gpu) << ")";
      LOG_INFO(logger_, step_oss.str());
    }
  }

  std::ostringstream summary_oss;
  summary_oss << "Cost Summary:\n"
              << "  MPPI Reported: " << total_reported_cost << "\n"
              << "  Host Recalc:   " << total_recalculated_cost << "\n"
              << "  GPU Recalc:    " << total_gpu_cost << "\n"
              << "  MPPI vs Host:  " << std::abs(total_reported_cost - total_recalculated_cost)
              << "\n"
              << "  MPPI vs GPU:   " << std::abs(total_reported_cost - total_gpu_cost) << "\n"
              << "  Host vs GPU:   " << std::abs(total_recalculated_cost - total_gpu_cost);
  LOG_INFO(logger_, summary_oss.str());
  LOG_INFO(logger_, "===============================================================");
#endif

  for (const auto& traj : sampled_trajectories) {
    auto* state_seq_proto = planning_info.add_sample_xs();
    for (int t = 0; t < traj.cols(); t += 4) {
      // downsample by 4 for brevity
      auto* traj_point = state_seq_proto->add_trajectory();
      traj_point->set_pos_x(traj(0, t));
      traj_point->set_pos_y(traj(1, t));
      traj_point->set_velocity(traj(2, t));
      traj_point->set_heading(traj(3, t));
      traj_point->set_accel(traj(4, t));
      traj_point->set_steer(traj(5, t));
    }
  }

  for (int i = 0; i < sampled_costs.size() && i < planning_info.sample_xs_size(); ++i) {
    planning_info.mutable_sample_xs(i)->set_cost(sampled_costs[i].col(0).sum());
  }

  LOG_DEBUG(logger_, "StochasticOptimizer get_debug_result cost time: {:.2f} ms",
            get_debug_result_tic.toc() * 1000.0);

  if (visualizer_) {
    visualizer_->log_planning_info(planning_info);
  }

  return planning_info;
}

template <int NUM_ROLLOUTS>
void StochasticOptimizer<NUM_ROLLOUTS>::set_parameters_to_proto(
    protos::config::SimulationConfig* sim_config_ptr) {
  sim_config_ptr->mutable_vehicle_info()->CopyFrom(vehicle_info_);
  sim_config_ptr->mutable_planning_parameters()->CopyFrom(planning_parameters_);
}

template <int NUM_ROLLOUTS>
void StochasticOptimizer<NUM_ROLLOUTS>::convert_parameters_to_proto(
    const TrajectoryCostParams& cost_params, const VehicleDynamics& vehicle_dynamics,
    const SAMPLER_T::SAMPLING_PARAMS_T& sampler_params) {
  vehicle_info_.Clear();
  planning_parameters_.Clear();

  // Vehicle Info
  vehicle_info_.set_wheel_base_m(cost_params.wheelbase);
  vehicle_info_.set_vehicle_width_m(cost_params.vehicle_width);
  vehicle_info_.set_vehicle_length_m(cost_params.vehicle_length);
  vehicle_info_.set_axle_to_front_bumper_m(cost_params.axle_to_front_bumper);
  vehicle_info_.set_axle_to_rear_bumper_m(cost_params.axle_to_rear_bumper);

  // Planning Parameters
  planning_parameters_.set_desired_speed(cost_params.target_velocity);
  planning_parameters_.set_mppi_lambda(mppi_controller_->getParams().lambda_);
  planning_parameters_.set_mppi_jerk_std(sampler_params.std_dev[0]);
  planning_parameters_.set_mppi_steer_rate_std(sampler_params.std_dev[1]);
  planning_parameters_.set_mppi_num_samples(NUM_ROLLOUTS);
  auto* cost_weights_ptr = planning_parameters_.mutable_cost_weight_coeff();
  cost_weights_ptr->set_position_weight(cost_params.position_coeff);
  cost_weights_ptr->set_velocity_weight(cost_params.velocity_coeff);
  cost_weights_ptr->set_heading_weight(cost_params.angle_coeff);
  cost_weights_ptr->set_accel_effort_weight(cost_params.accel_effort_coeff);
  cost_weights_ptr->set_steer_effort_weight(cost_params.steer_effort_coeff);
  cost_weights_ptr->set_jerk_effort_weight(cost_params.control_cost_coeff[0]);
  cost_weights_ptr->set_steer_rate_effort_weight(cost_params.control_cost_coeff[1]);
  auto* constraint_limits_ptr = planning_parameters_.mutable_constraint_limit();
  constraint_limits_ptr->set_max_accel_mps2(cost_params.max_accel);
  constraint_limits_ptr->set_min_accel_mps2(cost_params.min_accel);
  constraint_limits_ptr->set_min_jerk_mps3(vehicle_dynamics.control_rngs_[0].x);
  constraint_limits_ptr->set_max_jerk_mps3(vehicle_dynamics.control_rngs_[0].y);
  constraint_limits_ptr->set_max_steer_rate_rps(vehicle_dynamics.control_rngs_[1].y);
  constraint_limits_ptr->set_max_steer_angle_rad(cost_params.max_steer_angle);
  constraint_limits_ptr->set_longitudinal_safety_margin_m(cost_params.longitudinal_safety_margin);
  constraint_limits_ptr->set_lateral_safety_margin_m(cost_params.lateral_safety_margin);
}

template class StochasticOptimizer<1024>;
template class StochasticOptimizer<2048>;
template class StochasticOptimizer<4096>;
template class StochasticOptimizer<8192>;
template class StochasticOptimizer<16384>;
