# Use local robin-map instead of fetching from GitHub
if(NOT TARGET tsl::robin_map)
    # Get absolute path to robin-map in external directory
    get_filename_component(ROBIN_MAP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../external/robin-map" ABSOLUTE)
    message(STATUS "Using local robin-map from ${ROBIN_MAP_DIR}")

    # Header-only interface target
    add_library(tsl_robin_map INTERFACE)
    target_include_directories(tsl_robin_map INTERFACE
        $<BUILD_INTERFACE:${ROBIN_MAP_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    add_library(tsl::robin_map ALIAS tsl_robin_map)

    # Mark as populated for FetchContent
    set(tsl_robin_map_POPULATED TRUE CACHE BOOL "" FORCE)
    set(tsl_robin_map_SOURCE_DIR ${ROBIN_MAP_DIR} CACHE PATH "" FORCE)
endif()
