/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:18:52
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2026-01-20 23:42:38
 * @FilePath: /mppi-in-autonomous-driving/common/common.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

// Foxglove is optional - only needed for visualization
#ifdef USE_FOXGLOVE
#include "foxglove/schemas.hpp"
#endif
#include "common/protos/config.pb.h"

#include <yaml-cpp/yaml.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

#define LOG_TRACE(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::trace, __VA_ARGS__)
#define LOG_DEBUG(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::debug, __VA_ARGS__)
#define LOG_INFO(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::info, __VA_ARGS__)
#define LOG_WARN(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::warn, __VA_ARGS__)
#define LOG_ERROR(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::err, __VA_ARGS__)
#define LOG_CRITICAL(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::critical, __VA_ARGS__)

constexpr int kHorizonLength = 64;
constexpr double kTimeStepSec = 0.1;

/**
 * @brief Factory function to create a logger with console color sink
 * @param name Logger name
 * @param level Log level (trace, debug, info, warn, error, critical)
 * @return Shared pointer to the created logger
 */
inline std::shared_ptr<spdlog::logger> create_logger(const std::string& name,
                                                     const std::string& level = "info") {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto logger = std::make_shared<spdlog::logger>(name, console_sink);
  logger->set_level(spdlog::level::from_str(level));
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^\033[1m%l\033[0m%$] [%s:%#] %v");
  return logger;
}

struct ControlInput {
  double accel{0};  // longitudinal acceleration (m/s^2)
  double steer{0};  // front wheel steering angle (rad)
};

struct StateInfo {
  double x{0};         // position x (m)
  double y{0};         // position y (m)
  double velocity{0};  // velocity (m/s)
  double heading{0};   // heading angle (rad)
  double accel{0};     // acceleration (m/s^2)
  double steer{0};     // front wheel steering angle (rad)

#ifdef USE_FOXGLOVE
  foxglove::schemas::Pose to_pose() const {
    const double half_yaw = 0.5 * heading;
    foxglove::schemas::Pose pose;
    pose.position = foxglove::schemas::Vector3{x, y, 0.0};
    pose.orientation = foxglove::schemas::Quaternion{0.0, 0.0, sin(half_yaw), cos(half_yaw)};
    return pose;
  }
#endif
};

struct VehicleInfo {
  double width;
  double length;
  double wheel_base;
};

struct PathPoint {
  double x;
  double y;
  double yaw;
  double v;
  double t;
};

class TimeUtil {
public:
  static double NowSeconds() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    return duration_cast<microseconds>(duration).count() / 1e6;
  }

#ifdef USE_FOXGLOVE
  static foxglove::schemas::Timestamp NowTimestamp() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    auto sec = duration_cast<seconds>(duration).count();
    auto nsec = duration_cast<nanoseconds>(duration).count() % 1000000000;
    foxglove::schemas::Timestamp ts;
    ts.sec = static_cast<uint32_t>(sec);
    ts.nsec = static_cast<uint32_t>(nsec);
    return ts;
  }
#endif

  static std::string NowTimeString() {
    using namespace std::chrono;
    auto tp = system_clock::now();
    auto t = system_clock::to_time_t(tp);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    return oss.str();
  }
};

class TicToc {
public:
  TicToc(void) { tic(); }

  void tic(void) { start = std::chrono::system_clock::now(); }

  double toc(void) {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count();
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

/**
 * @brief Template function to convert a floating-point number to string with fixed decimal places
 * @tparam digits Number of decimal places to keep
 * @param value Input floating-point number
 * @return std::string String representation with specified decimal places
 */
template <uint8_t digits>
std::string to_fixed(double value) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(digits) << value;
  return oss.str();
}

protos::config::SimulationConfig LoadSimulationConfigFromYAML(const YAML::Node& config);