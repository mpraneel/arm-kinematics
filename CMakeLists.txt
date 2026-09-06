cmake_minimum_required(VERSION 3.16)
project(arm_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(ARM_BUILD_TESTS "Build the unit tests" ON)
option(ARM_BUILD_VIZ "Build the SFML visualizer" OFF)

find_package(Eigen3 REQUIRED NO_MODULE)

# The kinematics core knows nothing about rendering, so the benchmark harness
# can run headless.
add_library(arm_core
  src/kinematics.cpp
  src/ik.cpp
  src/collision.cpp
)
target_include_directories(arm_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(arm_core PUBLIC Eigen3::Eigen)
target_compile_options(arm_core PRIVATE -Wall -Wextra -Wpedantic)

if(ARM_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.1
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(arm_tests
    tests/test_kinematics.cpp
    tests/test_ik.cpp
    tests/test_collision.cpp
  )
  target_link_libraries(arm_tests PRIVATE arm_core Catch2::Catch2WithMain)

  enable_testing()
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
  include(Catch)
  catch_discover_tests(arm_tests)
endif()

if(ARM_BUILD_VIZ)
  add_subdirectory(viz)
endif()
