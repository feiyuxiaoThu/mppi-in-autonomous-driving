cmake_minimum_required(VERSION 3.20)
project(ContingencyPlanning LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CUDA_ARCHITECTURES 86)

# Set RPATH to find shared libraries in 3rdparty/foxglove/lib
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH "${PROJECT_SOURCE_DIR}/3rdparty/foxglove/lib")
# Suppress CUDA warnings from Eigen library interactions
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:-Xcudafe=--diag_suppress=20014>)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(ENABLE_WARNINGS "Enable strict compiler warnings" OFF)

# Dependencies
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

# Add local robin-map include path (before subdirectories)
include_directories(${PROJECT_SOURCE_DIR}/external/robin-map/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/environment-model)
set(MPPI_BUILD_TESTS OFF CACHE BOOL "Disable MPPI tests/internal libs" FORCE)
set(MPPI_BUILD_EXAMPLES OFF CACHE BOOL "Disable MPPI examples" FORCE)
set(MPPI_USE_CUDA_BARRIERS OFF CACHE BOOL "Disable CUDA barriers for older GPUs" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/MPPI-Generic)

# Protobuf: generate sources
# set(PROTO_FILES "${PROJECT_SOURCE_DIR}/common/protos/planning_info.proto")
file(GLOB PROTO_FILES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/common/protos/*.proto")
set(PROTO_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/common/protos")
set(PROTO_SRCS "")
set(PROTO_HDRS "")
foreach(PROTO_FILE IN LISTS PROTO_FILES)
    get_filename_component(PROTO_FILE_WE ${PROTO_FILE} NAME_WE)
    list(APPEND PROTO_SRCS "${PROTO_OUTPUT_DIR}/${PROTO_FILE_WE}.pb.cc")
    list(APPEND PROTO_HDRS "${PROTO_OUTPUT_DIR}/${PROTO_FILE_WE}.pb.h")
endforeach()
add_custom_command(
    OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROTO_OUTPUT_DIR} --proto_path=${PROTO_OUTPUT_DIR} ${PROTO_FILES}
    DEPENDS ${PROTO_FILES}
    COMMENT "Generating C++ from ${PROTO_FILES}"
    VERBATIM
)

# Create a custom target to ensure protobuf files are generated
add_custom_target(generate_proto DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})

# Foxglove SDK (headers + prebuilt static lib)
set(FOXGLOVE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/foxglove")
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${FOXGLOVE_SOURCE_DIR}/src/*.cpp"
    "${FOXGLOVE_SOURCE_DIR}/src/server/*.cpp"
)

add_library(foxglove_sdk SHARED IMPORTED)
set_target_properties(foxglove_sdk PROPERTIES
    IMPORTED_LOCATION "${FOXGLOVE_SOURCE_DIR}/lib/libfoxglove.so"
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${FOXGLOVE_SOURCE_DIR}/include"
)

# Sources
file(GLOB SIMULATOR_SOURCES CONFIGURE_DEPENDS "modules/simulator/*.cpp")
file(GLOB PLANNER_SOURCES CONFIGURE_DEPENDS
    "modules/planner/*.cpp"
    "modules/planner/*.cu"
)
file(GLOB VISUALIZER_SOURCES CONFIGURE_DEPENDS "modules/visualizer/*.cpp")
file(GLOB COMMON_SOURCES CONFIGURE_DEPENDS "common/*.cpp")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${FOXGLOVE_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/environment-model/include/)

add_library(visualizer SHARED ${VISUALIZER_SOURCES} ${COMMON_SOURCES})
target_compile_definitions(visualizer 
    PUBLIC 
        PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_include_directories(visualizer
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${FOXGLOVE_SOURCE_DIR}/include
)
target_link_libraries(visualizer
    PUBLIC
        EnvironmentModel::env_model
)

# Simulator library
add_library(simulator SHARED ${SIMULATOR_SOURCES} ${COMMON_SOURCES})
target_compile_definitions(simulator 
    PUBLIC 
        PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_include_directories(simulator
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)
target_link_libraries(simulator
    PUBLIC
        EnvironmentModel::env_model
        visualizer
)

add_library(planner SHARED ${PLANNER_SOURCES} ${COMMON_SOURCES})
set_target_properties(planner PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)
target_compile_definitions(planner
    PUBLIC
        USE_FOXGLOVE
        USE_VISUALIZER
)
target_include_directories(planner
    PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${PROJECT_SOURCE_DIR}/3rdparty/MPPI-Generic/include
        ${PROJECT_SOURCE_DIR}
        ${FOXGLOVE_SOURCE_DIR}/include
)
target_link_libraries(planner
    PUBLIC
        Eigen3::Eigen
        MPPI::MPPI
        EnvironmentModel::env_model
)

# Simple simulator (lightweight replacement for CommonRoad)
file(GLOB SIMPLE_SIMULATOR_SOURCES CONFIGURE_DEPENDS "modules/simple_simulator/*.cpp")
add_library(simple_simulator SHARED ${SIMPLE_SIMULATOR_SOURCES} ${COMMON_SOURCES})
target_include_directories(simple_simulator
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)
target_link_libraries(simple_simulator
    PUBLIC
        spdlog::spdlog
        yaml-cpp
)

# Standalone planner library (no simulation/visualization dependencies)
add_library(planner_standalone SHARED ${PLANNER_SOURCES} ${COMMON_SOURCES})
set_target_properties(planner_standalone PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)
target_include_directories(planner_standalone
    PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${PROJECT_SOURCE_DIR}/3rdparty/MPPI-Generic/include
        ${PROJECT_SOURCE_DIR}
)
target_link_libraries(planner_standalone
    PUBLIC
        Eigen3::Eigen
        MPPI::MPPI
)

# Executable
add_executable(planning_node modules/planning_node.cpp ${PROTO_SRCS})
add_dependencies(planning_node generate_proto)
target_sources(planning_node PRIVATE ${FOXGLOVE_SOURCES})
target_include_directories(planning_node
    PRIVATE
        ${PROTO_OUTPUT_DIR}
        ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(planning_node
    PRIVATE
        simulator
        planner
        visualizer
        spdlog::spdlog
        yaml-cpp
        foxglove_sdk
        Threads::Threads
        ${Protobuf_LIBRARIES}
        dl
)

if(ENABLE_WARNINGS)
    foreach(tgt simulator planning_node)
        target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()

# Standalone planning node (lightweight closed-loop simulation)
add_executable(planning_node_standalone modules/planning_node_standalone.cpp ${PROTO_SRCS})
add_dependencies(planning_node_standalone generate_proto)
target_include_directories(planning_node_standalone
    PRIVATE
        ${PROTO_OUTPUT_DIR}
        ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(planning_node_standalone
    PRIVATE
        planner_standalone
        simple_simulator
        spdlog::spdlog
        yaml-cpp
        Threads::Threads
        ${Protobuf_LIBRARIES}
)
