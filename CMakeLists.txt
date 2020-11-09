cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME mc_state_observation)
set(PROJECT_DESCRIPTION "Implementation of state-observation for mc_rtc")
set(PROJECT_URL "https://github.com/isri-aist/mc_state_observation")
set(PROJECT_VERSION 0.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(DOXYGEN_USE_MATHJAX "YES")
set(CMAKE_CXX_STANDARD 11)

option(BUILD_TESTS "Build unit tests" ON)

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES CXX)

find_package(mc_rtc 1.6 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(state-observation 1.3 REQUIRED)

add_subdirectory(src)

if(BUILD_TESTS)
  find_package(Boost COMPONENTS unit_test_framework REQUIRED)
  enable_testing()
  add_subdirectory(tests)
endif()
