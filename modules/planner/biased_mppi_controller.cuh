/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-30 00:00:00
 * @LastEditTime: 2026-03-30 00:00:00
 * @FilePath: /mppi-in-autonomous-driving/modules/planner/biased_mppi_controller.cuh
 * Copyright (c) 2026 by puyu, All Rights Reserved.
 */

#pragma once

#include "mppi/controllers/MPPI/mppi_controller.cuh"
#include "mppi/sampling_distributions/gaussian/gaussian.cuh"

namespace mppi {
namespace sampling_distributions {

// Define a local version of GaussianParams with higher MAX_DISTRIBUTIONS
// to avoid polluting the global 3rdparty library.
template <int C_DIM>
using BiasedGaussianParams = GaussianParamsImpl<C_DIM, 5>;

template <class DYN_PARAMS_T>
class BiasedGaussianDistribution
  : public GaussianDistributionImpl<BiasedGaussianDistribution<DYN_PARAMS_T>, BiasedGaussianParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<BiasedGaussianDistribution, BiasedGaussianParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  BiasedGaussianDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream) {}
  BiasedGaussianDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : PARENT_CLASS(params, stream) {}

  float* getControlSamplesDevicePtr() { return this->control_samples_d_; }
  float* getControlMeansDevicePtr() { return this->control_means_d_; }
  float* getStdDevDevicePtr() { return this->std_dev_d_; }
  const SAMPLING_PARAMS_T& getParams() const { return this->params_; }
};

} // namespace sampling_distributions
} // namespace mppi

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS,
          class SAMPLING_T = ::mppi::sampling_distributions::BiasedGaussianDistribution<typename DYN_T::DYN_PARAMS_T>,
          class PARAMS_T = ControllerParams<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM, MAX_TIMESTEPS>>
class BiasedMPPIController : public VanillaMPPIController<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T, PARAMS_T> {
public:
  using PARENT_CLASS = VanillaMPPIController<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T, PARAMS_T>;
  using state_array = typename PARENT_CLASS::state_array;
  using control_trajectory = typename PARENT_CLASS::control_trajectory;

  BiasedMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, float dt, int max_iter,
                       float lambda, float alpha, int num_timesteps = MAX_TIMESTEPS,
                       const Eigen::Ref<const control_trajectory>& init_control_traj = control_trajectory::Zero(),
                       cudaStream_t stream = nullptr)
      : PARENT_CLASS(model, cost, fb_controller, sampler, dt, max_iter, lambda, alpha, num_timesteps, init_control_traj, stream) {
    // FIX 1: Initialize alphas_ to avoid uninitialized read in single distribution path
    alphas_ = {1.0f};
  }

  void computeControl(const Eigen::Ref<const state_array>& state, int optimization_stride = 1) override;

  /**
   * @brief Set prior control sequences (from E2E trajectories)
   * @param priors Vector of control trajectories, size M. If empty, resets to Distribution 0 only.
   */
  void setPriors(const std::vector<control_trajectory>& priors);

  void setMixingCoefficients(const std::vector<float>& alphas) {
    if (alphas.size() != (size_t)this->sampler_->getNumDistributions()) {
        throw std::runtime_error("alphas size mismatch: expected " + 
                                 std::to_string(this->sampler_->getNumDistributions()) + 
                                 ", got " + std::to_string(alphas.size()));
    }
    alphas_ = alphas;
  }

protected:
  std::vector<float> alphas_;
};

#if __CUDACC__
#include "biased_mppi_controller.cu"
#endif
