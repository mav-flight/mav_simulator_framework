cmake_minimum_required(VERSION 3.5)
project(mav_simulator)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 2)
  # Default to C99
  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()
