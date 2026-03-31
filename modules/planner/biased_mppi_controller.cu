/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-30 00:00:00
 * @LastEditTime: 2026-03-30 00:00:00
 * @FilePath: /mppi-in-autonomous-driving/modules/planner/biased_mppi_controller.cu
 * Copyright (c) 2026 by puyu, All Rights Reserved.
 */

#include "biased_mppi_controller.cuh"
#include <mppi/utils/cuda_math_utils.cuh>
#include <numeric>

namespace mppi {
namespace kernels {

/**
 * @brief CUDA kernel to correct weights for Biased-MPPI using Log-Sum-Exp.
 */
__global__ void BiasedWeightCorrectionKernel(
    float* trajectory_costs_d,
    const float* control_samples_d,
    const float* control_means_d,
    const float* std_dev_d,
    const float* alphas_d,
    float lambda,
    int num_distributions,
    int num_rollouts_per_dist,
    int num_timesteps,
    int control_dim) {
  
  int rollout_idx_in_dist = blockIdx.x * blockDim.x + threadIdx.x;
  int dist_idx = threadIdx.z;
  
  if (rollout_idx_in_dist >= num_rollouts_per_dist) return;

  int global_rollout_idx = dist_idx * num_rollouts_per_dist + rollout_idx_in_dist;
  
  const float* u_i = &control_samples_d[global_rollout_idx * num_timesteps * control_dim];

  float log_q_m[16]; 
  float max_log_q = -1e30f;

  for (int m = 0; m < num_distributions; ++m) {
    float log_likelihood = 0.0f;
    const float* mu_m = &control_means_d[m * num_timesteps * control_dim];
    const float* sigma_shared = std_dev_d; 

    for (int t = 0; t < num_timesteps; ++t) {
      for (int c = 0; c < control_dim; ++c) {
        float diff = u_i[t * control_dim + c] - mu_m[t * control_dim + c];
        float sigma = fmaxf(sigma_shared[c], 1e-3f); 
        log_likelihood -= 0.5f * (diff * diff) / (sigma * sigma);
      }
    }
    
    log_q_m[m] = logf(fmaxf(alphas_d[m], 1e-6f)) + log_likelihood;
    if (log_q_m[m] > max_log_q) max_log_q = log_q_m[m];
  }

  float sum_exp = 0.0f;
  for (int m = 0; m < num_distributions; ++m) {
    sum_exp += expf(log_q_m[m] - max_log_q);
  }
  float log_q_mix = max_log_q + logf(sum_exp);

  float log_p_nominal = log_q_m[0] - logf(fmaxf(alphas_d[0], 1e-6f));

  trajectory_costs_d[global_rollout_idx] += lambda * (log_p_nominal - log_q_mix);
}

} // namespace kernels
} // namespace mppi

#define BIASED_MPPI_TEMPLATE \
    template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, class SAMPLING_T, class PARAMS_T>

#define BiasedMPPI BiasedMPPIController<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T, PARAMS_T>

BIASED_MPPI_TEMPLATE
void BiasedMPPI::setPriors(const std::vector<control_trajectory>& priors) {
    int m = priors.size();
    int total_dist = m + 1;
    if (total_dist > SAMPLING_T::SAMPLING_PARAMS_T::MAX_DISTRIBUTIONS) {
        throw std::runtime_error("Number of priors exceeds MAX_DISTRIBUTIONS");
    }
    
    if (this->sampler_->getNumDistributions() != total_dist) {
        // MUST reallocate sampler memory when distribution count changes
        this->sampler_->freeCudaMem();
        this->sampler_->setNumDistributions(total_dist);
        this->sampler_->allocateCUDAMemoryHelper();

        // Also reallocate controller memory (trajectory_costs_d_, etc.)
        this->deallocateCUDAMemory();
        this->allocateCUDAMemoryHelper(total_dist - 1); 
    }
    
    for (int i = 0; i < m; ++i) {
        this->sampler_->copyImportanceSamplerToDevice(priors[i].data(), i + 1, false);
    }
    
    // Auto-generate alpha weights if they don't match the new distribution count
    if (alphas_.size() != (size_t)total_dist) {
        std::vector<float> default_alphas(total_dist, 0.8f / m);
        if (total_dist == 1) default_alphas = {1.0f};
        else default_alphas[0] = 0.2f;
        this->setMixingCoefficients(default_alphas);
    }
}

BIASED_MPPI_TEMPLATE
void BiasedMPPI::computeControl(const Eigen::Ref<const state_array>& state, int optimization_stride) {
  this->free_energy_statistics_.real_sys.previousBaseline = this->getBaselineCost();

  HANDLE_ERROR(cudaMemcpyAsync(this->initial_state_d_, state.data(), DYN_T::STATE_DIM * sizeof(float),
                               cudaMemcpyHostToDevice, this->stream_));

  int num_dist = this->sampler_->getNumDistributions();
  int rollouts_per_dist = NUM_ROLLOUTS; 
  int total_rollouts = num_dist * rollouts_per_dist;

  // Use a local host vector for all rollout costs
  std::vector<float> h_trajectory_costs(total_rollouts);
  
  // Ensure alphas_d_ is ready (should be handled by setMixingCoefficients)
  if (!alphas_d_) {
      std::vector<float> default_alphas(num_dist, 1.0f/num_dist);
      this->setMixingCoefficients(default_alphas);
  }

  float* control_samples_d = this->sampler_->getControlSamplesDevicePtr();
  float* control_means_d = this->sampler_->getControlMeansDevicePtr();
  float* std_dev_d = this->sampler_->getStdDevDevicePtr();

  for (int opt_iter = 0; opt_iter < this->getNumIters(); opt_iter++) {
    this->copyNominalControlToDevice(false);
    this->sampler_->generateSamples(optimization_stride, opt_iter, this->gen_, false);

    dim3 dimBlock = this->params_.dynamics_rollout_dim_;
    dim3 dimGrid(mppi::math::int_ceil(rollouts_per_dist, dimBlock.x), 1, 1);
    dimBlock.z = num_dist; 
    
    mppi::kernels::launchRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
        this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), rollouts_per_dist,
        this->getLambda(), this->getAlpha(), this->initial_state_d_, this->trajectory_costs_d_,
        dimBlock, this->stream_, false);

    mppi::kernels::BiasedWeightCorrectionKernel<<<dimGrid, dimBlock, 0, this->stream_>>>(
        this->trajectory_costs_d_,
        control_samples_d,
        control_means_d,
        std_dev_d,
        alphas_d_,
        this->getLambda(),
        num_dist,
        rollouts_per_dist,
        this->getNumTimesteps(),
        DYN_T::CONTROL_DIM
    );

    // CRITICAL FIX: Only use the first NUM_ROLLOUTS (Distribution 0) for baseline and normalizer calculation.
    // Standard MPPI reduction kernels require power-of-2 input sizes (e.g. 4096).
    // Using total_rollouts (e.g. 3*4096) causes misaligned address errors in樹状规约 kernels.
    int aligned_rollouts = NUM_ROLLOUTS;

    HANDLE_ERROR(cudaMemcpyAsync(h_trajectory_costs.data(), this->trajectory_costs_d_,
                                 aligned_rollouts * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    float baseline = mppi::kernels::computeBaselineCost(h_trajectory_costs.data(), aligned_rollouts);
    this->setBaseline(baseline);
    
    mppi::kernels::launchNormExpKernel(aligned_rollouts, this->getNormExpThreads(), this->trajectory_costs_d_,
                                       1.0 / this->getLambda(), this->getBaselineCost(), this->stream_, false);
    
    HANDLE_ERROR(cudaMemcpyAsync(h_trajectory_costs.data(), this->trajectory_costs_d_,
                                 aligned_rollouts * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    float normalizer = mppi::kernels::computeNormalizer(h_trajectory_costs.data(), aligned_rollouts);
    this->setNormalizer(normalizer);
    
    // UPDATE LOGIC: Perform global weighted reduction across Distribution 0 rollouts
    mppi::kernels::launchWeightedReductionKernel<DYN_T::CONTROL_DIM>(
        this->trajectory_costs_d_,
        control_samples_d, 
        control_means_d, // Update Distribution 0 mean
        this->getNormalizerCost(),
        this->getNumTimesteps(),
        aligned_rollouts,
        this->sampler_->getParams().sum_strides,
        this->stream_,
        false
    );

    this->sampler_->setHostOptimalControlSequence(this->control_.data(), 0, true);
  }

  this->smoothControlTrajectory();
  this->computeStateTrajectory(state);
  
  state_array zero_state = this->model_->getZeroState();
  for (int i = 0; i < this->getNumTimesteps(); i++) {
    this->model_->enforceConstraints(zero_state, this->control_.col(i));
  }
}
