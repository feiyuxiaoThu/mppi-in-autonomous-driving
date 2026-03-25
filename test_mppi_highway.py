#!/usr/bin/env python3
"""
MPPI + HighwayEnv Integration Test
"""

import sys
sys.path.insert(0, '/app/mppi-in-autonomous-driving/build')

import gymnasium as gym
import highway_env
import mppi_core
import numpy as np

# Load MPPI config
with open('/app/mppi-in-autonomous-driving/config/default.yaml') as f:
    config_yaml = f.read()

# Create MPPI planner
planner = mppi_core.MPPIPlanner16384(config_yaml)

# Create HighwayEnv
env = gym.make('highway-v0', render_mode='rgb_array')
obs, info = env.reset()

print(f"Observation shape: {obs.shape}")
print(f"Observation: {obs[:3]}")  # First 3 vehicles

# HighwayEnv config
env.configure({
    "observation": {
        "type": "Kinematics",
        "vehicles_count": 15,
        "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
        "features_range": {
            "x": [-100, 100],
            "y": [-100, 100],
            "vx": [-20, 20],
            "vy": [-20, 20],
        },
        "absolute": True,
        "order": "sorted",
    },
    "action": {
        "type": "ContinuousAction",
    },
    "duration": 60,
})
obs, info = env.reset()

def highway_to_mppi_state(obs, env):
    """Convert HighwayEnv observation to MPPI StateInfo."""
    ego = mppi_core.StateInfo()
    
    # Ego vehicle is at index 0
    ego.x = obs[0, 1]  # x position
    ego.y = obs[0, 2]  # y position
    ego.velocity = np.sqrt(obs[0, 3]**2 + obs[0, 4]**2)  # speed
    ego.heading = np.arctan2(obs[0, 6], obs[0, 5])  # heading from sin/cos
    ego.accel = 0.0
    ego.steer = 0.0
    
    return ego

def extract_obstacles(obs):
    """Extract obstacle list from HighwayEnv observation."""
    obstacles = []
    for i in range(1, len(obs)):  # Skip ego vehicle (index 0)
        if obs[i, 0] > 0.5:  # presence > 0.5
            obs_obj = mppi_core.Obstacle()
            obs_obj.id = f"vehicle_{i}"
            obs_obj.x = obs[i, 1]
            obs_obj.y = obs[i, 2]
            vx, vy = obs[i, 3], obs[i, 4]
            obs_obj.heading = np.arctan2(obs[i, 6], obs[i, 5])
            obs_obj.velocity = np.sqrt(vx**2 + vy**2)
            obs_obj.width = 2.0
            obs_obj.length = 5.0
            obs_obj.is_static = False
            obs_obj.type = 0  # VEHICLE
            obstacles.append(obs_obj)
    return obstacles

def generate_reference_line(ego_x, ego_y, ego_heading, length=100, step=2.0):
    """Generate a simple reference line ahead of ego vehicle."""
    ref_line = []
    for i in range(int(length / step)):
        x = ego_x + i * step * np.cos(ego_heading)
        y = ego_y + i * step * np.sin(ego_heading)
        ref_line.append([x, y])
    return ref_line

# Run simulation
print("\nStarting MPPI + HighwayEnv simulation...")
for step in range(100):
    # Convert state
    ego_state = highway_to_mppi_state(obs, env)
    
    # Generate reference line
    ref_line = generate_reference_line(ego_state.x, ego_state.y, ego_state.heading)
    
    # Extract obstacles
    obstacles = extract_obstacles(obs)
    
    # Plan with MPPI
    control = planner.plan_once(ego_state, ref_line, obstacles)
    
    # Convert MPPI control to HighwayEnv action
    # HighwayEnv uses [acceleration, steering] normalized roughly in [-1, 1]
    action = np.array([control.accel / 5.0, control.steer * 5.0])  # Scale appropriately
    action = np.clip(action, -1, 1)
    
    # Step environment
    obs, reward, terminated, truncated, info = env.step(action)
    
    if step % 10 == 0:
        print(f"Step {step}: x={ego_state.x:.1f}, y={ego_state.y:.1f}, "
              f"v={ego_state.velocity:.1f}, accel={control.accel:.2f}, steer={control.steer:.3f}")
    
    if terminated or truncated:
        print(f"Episode ended at step {step}")
        break

env.close()
print("\nSimulation completed!")
