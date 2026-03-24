# NOTE: The upper CMake version bound specified here does not prevent using newer
# CMake versions - rather, it simply tells CMake that we are aware of versions
# in this range, allowing CMake to adapt its behaviour accordingly.
#
# Citing the documentation:
# The cmake_minimum_required(VERSION) command implicitly invokes the cmake_policy(VERSION)
# command to specify that the current project code is written for the given range of CMake versions.
# Source: https://cmake.org/cmake/help/v3.25/command/cmake_minimum_required.html
cmake_minimum_required(VERSION 3.20..4.0)

project(EnvironmentModel
    LANGUAGES C CXX
    VERSION 0.1.0
    HOMEPAGE_URL "https://commonroad.in.tum.de/"
    DESCRIPTION "Classes and methods to represent the CommonRoad format in C++17")

# CMP0077 (3.13) - option() honors normal variables.
# Relevant for Eigen3
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# Set the minimum CMake policy version to 3.5 - required for cmake >= 4.0
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# CMP0126 (3.21) - Removal of normal variables by set(CACHE)
if(POLICY CMP0126)
    cmake_policy(SET CMP0126 NEW)
endif()

# CMP0135 - URL download timestamp
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# CMP0135 - FindBoost removed
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Adapted from Eigen3 - snippet to get a value for PROJECT_IS_TOP_LEVEL
# on CMake versions before v3.21.0
if(CMAKE_VERSION VERSION_LESS 3.21.0)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PROJECT_IS_TOP_LEVEL ON)
  else()
    set(PROJECT_IS_TOP_LEVEL OFF)
  endif()
  set(${PROJECT_NAME}_IS_TOP_LEVEL ${PROJECT_IS_TOP_LEVEL})
endif()

set(CMAKE_VERIFY_INTERFACE_HEADER_SETS ON)


# Ugly hack to make CMake discover libomp from Homebrew on Github Actions
if(APPLE)

    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
        set(MAC_LIBOMP_PATH /opt/homebrew/opt/libomp)
    else()
        set(MAC_LIBOMP_PATH /usr/local/opt/libomp)
    endif()

    if(NOT EXISTS ${MAC_LIBOMP_PATH})
        message(FATAL_ERROR "could not find OpenMP path!")
    endif()

    list(APPEND CMAKE_PREFIX_PATH ${MAC_LIBOMP_PATH})

    message(STATUS "OpenMP prefix: ${MAC_LIBOMP_PATH}")

    find_path(_omp_include_dir
            NAMES omp.h
            PATHS ${_omp_path}/include
            REQUIRED
    )
    message(STATUS "OpenMP include dir: ${_omp_include_dir}")

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_${_lang}_COMPILER_ID MATCHES "AppleClang")
        foreach(_lang IN ITEMS C CXX)
            set(OpenMP_${_lang}_LIB_NAMES "omp")
            set(OpenMP_${_lang}_FLAGS "-Xclang -fopenmp")
            set(OpenMP_${_lang}_INCLUDE_DIR ${MAC_LIBOMP_PATH}/include)
        endforeach()
    endif()

    find_library(OpenMP_omp_LIBRARY
        NAMES omp
        PATHS ${MAC_LIBOMP_PATH}/lib
    )

    set(CMAKE_DISABLE_PRECOMPILE_HEADERS ON)
endif()

# Disable PCH on platforms other than Clang on Linux (spotty support)
if(NOT LINUX OR NOT (CMAKE_CXX_COMPILER_ID STREQUAL "Clang") OR DEFINED ENV{CIBUILDWHEEL})
    set(CMAKE_DISABLE_PRECOMPILE_HEADERS ON)
endif()

if (APPLE)
    # Disable shared libraries on apple --> Causes linker error with omp
    message(STATUS "Setting BUILD_SHARED_LIBS=OFF on MacOS" )
    set(BUILD_SHARED_LIBS OFF)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE=1")
endif()

set(CMAKE_COLOR_DIAGNOSTICS ON)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX("filesystem" COMPILER_SUPPORTS_CXX_FILESYSTEM)
if(NOT COMPILER_SUPPORTS_CXX_FILESYSTEM)
    message(FATAL_ERROR "Your C++ standard library is missing the <filesystem> header "
        "which is required to build the Environment Model. Please try to install a newer "
        "C++ standard library if possible and try again.\n"
        "Remember to start over with a fresh build directory, for example by invoking CMake "
        "with the \"--fresh\" flag.")
endif()

# Some extra debugging for project developers - safe to disable, but read on why they might be useful:

# By enabling CMAKE_LINK_LIBRARIES_ONLY_TARGETS, CMake will report errors whenever
# a name that does not refer to a target known to CMake is passed to
# target_link_libraries and friends.
# We use targets for all dependencies in this project, so if a non-target name is
# passed it usually indicates a bug (usually a typo or a missing find_package call)
# in the CMake configuration.
# Example:
#   target_link_libraries(example_library PUBLIC spdlog)
# This is incorrect as spdlog is not always a target name provided by the spdlog project - it might
# exist in the project itself when it is included via FetchContent, but not if we use a system-provided
# spdlog version.
# Normally, CMake would go on to add "spdlog" as a literal library to the linker command line (e.g. -lspdlog),
# which is not what we want as it might not correct to the correct spdlog library *and* it does not ensure
# the usage requirements (include directories, definitions, other compiler options) are correctly added
# to the compiler command line for example_library.
#
# But since we have CMAKE_LINK_LIBRARIES_ONLY_TARGETS enabled, CMake will instead print an error like this:
# CMake Error at CMakeLists.txt:123456 (target_link_libraries):
#   Target "example_library" has LINK_LIBRARIES_ONLY_TARGETS enabled, but it
#   links to:
#
#     spdlog
#
#   which is not a target.  Possible reasons include:
#
#     * There is a typo in the target name.
#     * A find_package call is missing for an IMPORTED target.
#     * An ALIAS target is missing.

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

set(CMAKE_MESSAGE_CONTEXT_SHOW ON)

# Compile command database is required for Clang-assisted parsing in Doxygen
# TODO: Set this conditionally
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Ensure executables are in the top level directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(CMakeDependentOption)

set(ENV_MODEL_BUILD_EXTRAS_DEFAULT ON)
if(NOT EnvironmentModel_IS_TOP_LEVEL)
    set(ENV_MODEL_BUILD_EXTRAS_DEFAULT OFF)
endif()

# TODO Move below dependency discovery
find_package(Doxygen QUIET COMPONENTS dot)

cmake_dependent_option(ENV_MODEL_BUILD_DOXYGEN
    "Build doxygen"
    ${ENV_MODEL_BUILD_EXTRAS_DEFAULT}
    "Doxygen_FOUND;NOT SKBUILD"
    OFF)

# TODO Maybe involve BUILD_TESTING here?
cmake_dependent_option(ENV_MODEL_BUILD_TESTS
    "Build tests"
    ${ENV_MODEL_BUILD_EXTRAS_DEFAULT}
    "NOT SKBUILD"
    OFF)

option(BUILD_SHARED_LIBS "Build environment model as a shared library" ON)

option(ENV_MODEL_BUILD_SHARED_LIBS "Build using shared libraries" ${BUILD_SHARED_LIBS})

cmake_dependent_option(ENV_MODEL_BUILD_CODE_COVERAGE
    "Build code coverage"
    OFF
    "ENV_MODEL_BUILD_TESTS;NOT SKBUILD"
    OFF)

# NOTE: Depends on Boost::program_options, but we already handle the case where
# Boost::program_options is not found below with an explicit warning for the user,
# so no need to restrict ENV_MODEL_BUILD_EXECUTABLE further here.
# cmake_dependent_option(ENV_MODEL_BUILD_EXECUTABLE
#     "Build executable"
#     ${ENV_MODEL_BUILD_EXTRAS_DEFAULT}
#     "NOT SKBUILD"
#     OFF)

# option(ENV_MODEL_BUILD_PYTHON_BINDINGS
#         "Build Python bindings"
#         ${ENV_MODEL_BUILD_EXTRAS_DEFAULT})

set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE OFF)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24.0)
    set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE ON)
else()
    message(WARNING "Your CMake version (${CMAKE_VERSION}) does not support "
        "the FetchContent find_package integration introduced in CMake 3.24. "
        "As a fallback, we will simply build all dependencies ourselves "
        "irrespective of whether a suitable system version exists. "
        "While this does not impair functionality, it might slow down the build "
        "process a bit.\n"
        "In case you have all required dependencies installed, you can try "
        "enabling the option\n"
        "\tENV_MODEL_SYSTEM_PACKAGES_FORCE\n"
        "which will force using find_package for all dependencies.")
endif()

option(ENV_MODEL_SYSTEM_PACKAGES "Try to use system packages for dependencies" ON)
cmake_dependent_option(ENV_MODEL_SYSTEM_PACKAGES_FORCE
    "For CMake<3.24: Force using system packages for all dependencies"
    OFF
    "NOT CMAKE_SUPPORTS_TRY_FIND_PACKAGE"
    OFF
    )

include(FetchContent)
FetchContent_Declare(
    commonroad_cmake
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../external/commonroad-cmake
)
FetchContent_MakeAvailable(commonroad_cmake)

list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

include(toolchain/DiscoverLLD OPTIONAL)
include(toolchain/DiscoverSanitizers OPTIONAL)

# This is a helper script that will automatically add a .gitignore file to the
# binary directory (build directory) so you don't have to do add every build folder
# to your .gitignore.
include(extras/GitIgnoreBinaryDir OPTIONAL)

if(DEFINED ENV{CIBUILDWHEEL} AND CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    # Ugly hack for broken pthread detection on manylinux2014_i686
    find_library(OpenMP_pthread_LIBRARY NAMES "pthread")
endif()

# Required for proper pthread discovery on some systems
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(OpenMP)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external)

# We have custom logic to check for system Boost/Protobuf, therefore we include
# ExternalBoost and ExternalProtobuf unconditionally
include(external/ExternalBoost)

include(external/ExternalProtobuf)

if(CMAKE_SUPPORTS_TRY_FIND_PACKAGE)
    if(SKBUILD_SELF_CONTAINED)
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
    endif()
    if(ENV_MODEL_SYSTEM_PACKAGES)
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE OPT_IN)
    else()
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
    endif()
endif()

# Include Eigen3, Boost filesystem, spdlog, and OpenMP
if(ENV_MODEL_SYSTEM_PACKAGES_FORCE)
    # This is the fallback branch in case the CMake version is older than 3.24
    # and the user requested we try to use system packages
    # For CMake > 3.24, fallback is automatic through the FetchContent find_package
    # integration.

    message(WARNING "ENV_MODEL_SYSTEM_PACKAGES_FORCE is set - trying to satisfy "
        "all dependencies using installed system packages.\n"
        "If this fails, consider disabling ENV_MODEL_SYSTEM_PACKAGES_FORCE and "
        "trying again.")

    find_package(Eigen3 3.3.7 REQUIRED)
    find_package(pugixml 1.11 REQUIRED)
    find_package(spdlog 1.8.0 REQUIRED)
    # find_package(GTest)
    # find_package(yaml-cpp 0.6.0 REQUIRED)
    find_package(range-v3 0.12.0 REQUIRED)

    # yaml-cpp::yaml-cpp is not present in installed config file as of version 0.7.0
    # if(NOT TARGET yaml-cpp::yaml-cpp)
    #     add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
    # endif()
else()
    # Normal path: We try to use find_package via FetchContent, otherwise we fall
    # back to normal FetchContent

    # include(external/ExternalEigen)
    include(external/ExternalPugixml)
    # include(external/ExternalSpdlog)
    # include(external/ExternalGoogleTest)
    # include(external/ExternalYamlCpp)
    include(external/ExternalRange)
endif()

include(external/ExternalTslRobinMap)

include(ExternalCLCS)

add_subdirectory(include/commonroad_cpp/interfaces/commonroad/protobufFormat)

# Add Environment model node
add_subdirectory(src)

# if(ENV_MODEL_BUILD_EXECUTABLE)
#     if(TARGET Boost::program_options AND TARGET OpenMP::OpenMP_CXX)
#         add_subdirectory(example)
#     elseif(NOT TARGET Boost::program_options)
#         message(WARNING "disabling example because Boost::program_options was not found")
#     elseif(NOT TARGET OpenMP::OpenMP_CXX)
#         message(WARNING "disabling example because OpenMP was not found")
#     endif()
# endif()

## Doxygen
if(ENV_MODEL_BUILD_DOXYGEN)
    include(extras/Doxygen)
endif()

# add unit testing
# if(ENV_MODEL_BUILD_TESTS)
#     if(TARGET Boost::filesystem AND TARGET OpenMP::OpenMP_CXX)
#         enable_testing()
#         add_subdirectory(tests)
#     elseif(NOT TARGET Boost::filesystem)
#         message(WARNING "disabling tests because Boost::program_options was not found")
#     elseif(NOT TARGET OpenMP::OpenMP_CXX)
#         message(WARNING "disabling tests because OpenMP was not found")
#     endif()
# endif()

include(cmake/install.cmake)

include(utils/EnsureStatic)

# Sanity check: Ensure we are building all dependencies as static libraries
if(SKBUILD)
    if(SKBUILD_SELF_CONTAINED)
        # If we are building all dependencies ourselves, then they should
        # all be static.

        ensure_all_static(env_model)

        # ensure_static(env_model)
        # ensure_static(yaml-cpp)
        # ensure_static(spdlog::spdlog)
        # ensure_static(pugixml-static)
        # ensure_static(protobuf::libprotobuf)
    else()
        # If we are partially using system libraries, then only the env_model library
        # itself has to be static.

        ensure_static(env_model)
    endif()
endif()
