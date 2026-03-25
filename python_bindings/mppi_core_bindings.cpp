/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-03-24
 * @Description: pybind11 bindings for MPPI core
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/reference_line.hpp"
#include "modules/planner/stochastic_optimizer.cuh"

namespace py = pybind11;

// Simple wrapper for StateInfo to make it easier to use from Python
struct PyStateInfo {
    double x = 0.0;
    double y = 0.0;
    double velocity = 0.0;
    double heading = 0.0;
    double accel = 0.0;
    double steer = 0.0;
    
    StateInfo to_state_info() const {
        StateInfo s;
        s.x = x;
        s.y = y;
        s.velocity = velocity;
        s.heading = heading;
        s.accel = accel;
        s.steer = steer;
        return s;
    }
    
    static PyStateInfo from_state_info(const StateInfo& s) {
        PyStateInfo py_s;
        py_s.x = s.x;
        py_s.y = s.y;
        py_s.velocity = s.velocity;
        py_s.heading = s.heading;
        py_s.accel = s.accel;
        py_s.steer = s.steer;
        return py_s;
    }
};

struct PyControlInput {
    double accel = 0.0;
    double steer = 0.0;
    
    ControlInput to_control_input() const {
        ControlInput c;
        c.accel = accel;
        c.steer = steer;
        return c;
    }
    
    static PyControlInput from_control_input(const ControlInput& c) {
        PyControlInput py_c;
        py_c.accel = c.accel;
        py_c.steer = c.steer;
        return py_c;
    }
};

struct PyObstacle {
    std::string id;
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double width = 1.8;
    double length = 4.5;
    bool is_static = false;
    int type = 0;  // 0: VEHICLE, 1: PEDESTRIAN, etc.
    std::vector<PathPoint> prediction;
    
    common::Obstacle to_obstacle() const {
        common::Obstacle obs;
        obs.set_id(id);
        obs.set_x(x);
        obs.set_y(y);
        obs.set_heading(heading);
        obs.set_width(width);
        obs.set_length(length);
        obs.set_is_static(is_static);
        obs.set_type(static_cast<common::ObstacleType>(type));
        obs.set_prediction(prediction);
        return obs;
    }
};

// Wrapper for MPPI planner with fixed rollout number
template<int NUM_ROLLOUTS>
class MPPIPlannerWrapper {
public:
    MPPIPlannerWrapper(const std::string& config_yaml) {
        // Parse config from YAML string
        config_ = YAML::Load(config_yaml);
        planner_ = std::make_unique<StochasticOptimizer<NUM_ROLLOUTS>>(config_);
    }
    
    PyControlInput plan_once(const PyStateInfo& ego_state,
                              const std::vector<std::vector<double>>& ref_line_points,
                              const std::vector<PyObstacle>& obstacles) {
        // Convert reference line
        std::vector<double> x_seq, y_seq;
        for (const auto& point : ref_line_points) {
            if (point.size() >= 2) {
                x_seq.push_back(point[0]);
                y_seq.push_back(point[1]);
            }
        }
        auto ref_line = std::make_shared<ReferenceLine>(x_seq, y_seq, 0.2);
        
        // Convert obstacles
        auto obstacle_list = std::make_shared<common::ObstacleList>();
        for (const auto& py_obs : obstacles) {
            obstacle_list->append(py_obs.to_obstacle());
        }
        
        // Plan
        auto control = planner_->plan_once(ego_state.to_state_info(), ref_line, obstacle_list);
        return PyControlInput::from_control_input(control);
    }
    
    std::vector<std::vector<double>> get_optimized_trajectory() {
        auto traj = planner_->get_optimized_trajectory();
        std::vector<std::vector<double>> result;
        for (int i = 0; i < traj.rows(); ++i) {
            result.push_back({traj(i, 0), traj(i, 1)});
        }
        return result;
    }
    
private:
    YAML::Node config_;
    std::unique_ptr<StochasticOptimizer<NUM_ROLLOUTS>> planner_;
};

PYBIND11_MODULE(mppi_core, m) {
    m.doc() = "MPPI Core Planner for Autonomous Driving";
    
    // Bind StateInfo
    py::class_<PyStateInfo>(m, "StateInfo")
        .def(py::init<>())
        .def_readwrite("x", &PyStateInfo::x)
        .def_readwrite("y", &PyStateInfo::y)
        .def_readwrite("velocity", &PyStateInfo::velocity)
        .def_readwrite("heading", &PyStateInfo::heading)
        .def_readwrite("accel", &PyStateInfo::accel)
        .def_readwrite("steer", &PyStateInfo::steer);
    
    // Bind ControlInput
    py::class_<PyControlInput>(m, "ControlInput")
        .def(py::init<>())
        .def_readwrite("accel", &PyControlInput::accel)
        .def_readwrite("steer", &PyControlInput::steer);
    
    // Bind Obstacle
    py::class_<PyObstacle>(m, "Obstacle")
        .def(py::init<>())
        .def_readwrite("id", &PyObstacle::id)
        .def_readwrite("x", &PyObstacle::x)
        .def_readwrite("y", &PyObstacle::y)
        .def_readwrite("heading", &PyObstacle::heading)
        .def_readwrite("width", &PyObstacle::width)
        .def_readwrite("length", &PyObstacle::length)
        .def_readwrite("is_static", &PyObstacle::is_static)
        .def_readwrite("type", &PyObstacle::type)
        .def_readwrite("prediction", &PyObstacle::prediction);
    
    // Bind PathPoint
    py::class_<PathPoint>(m, "PathPoint")
        .def(py::init<>())
        .def_readwrite("x", &PathPoint::x)
        .def_readwrite("y", &PathPoint::y)
        .def_readwrite("yaw", &PathPoint::yaw)
        .def_readwrite("v", &PathPoint::v)
        .def_readwrite("t", &PathPoint::t);
    
    // Bind MPPI Planner with different rollout numbers
    py::class_<MPPIPlannerWrapper<1024>>(m, "MPPIPlanner1024")
        .def(py::init<const std::string&>())
        .def("plan_once", &MPPIPlannerWrapper<1024>::plan_once)
        .def("get_optimized_trajectory", &MPPIPlannerWrapper<1024>::get_optimized_trajectory);
        
    py::class_<MPPIPlannerWrapper<2048>>(m, "MPPIPlanner2048")
        .def(py::init<const std::string&>())
        .def("plan_once", &MPPIPlannerWrapper<2048>::plan_once)
        .def("get_optimized_trajectory", &MPPIPlannerWrapper<2048>::get_optimized_trajectory);
        
    py::class_<MPPIPlannerWrapper<4096>>(m, "MPPIPlanner4096")
        .def(py::init<const std::string&>())
        .def("plan_once", &MPPIPlannerWrapper<4096>::plan_once)
        .def("get_optimized_trajectory", &MPPIPlannerWrapper<4096>::get_optimized_trajectory);
        
    py::class_<MPPIPlannerWrapper<8192>>(m, "MPPIPlanner8192")
        .def(py::init<const std::string&>())
        .def("plan_once", &MPPIPlannerWrapper<8192>::plan_once)
        .def("get_optimized_trajectory", &MPPIPlannerWrapper<8192>::get_optimized_trajectory);
        
    py::class_<MPPIPlannerWrapper<16384>>(m, "MPPIPlanner16384")
        .def(py::init<const std::string&>())
        .def("plan_once", &MPPIPlannerWrapper<16384>::plan_once)
        .def("get_optimized_trajectory", &MPPIPlannerWrapper<16384>::get_optimized_trajectory);
}