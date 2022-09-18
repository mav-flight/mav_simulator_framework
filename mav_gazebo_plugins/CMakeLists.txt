cmake_minimum_required(VERSION 3.5)
project(mav_gazebo_plugins)

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
  find_package(gazebo_dev REQUIRED)
  find_package(gazebo_ros REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

  # =============== gazebo_ros_motor_model plugin ===============
  add_library(gazebo_ros_motor_model SHARED
    src/mav_gazebo_plugins/gazebo_ros_motor_model.cpp
  )
  target_include_directories(
    gazebo_ros_motor_model PUBLIC include
  )
  ament_target_dependencies(gazebo_ros_motor_model
    gazebo_dev
    gazebo_ros
    rclcpp
    std_msgs
  )
  ament_export_libraries(gazebo_ros_motor_model)
  install(TARGETS
      gazebo_ros_motor_model
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  if(ENABLE_PROFILER)
    target_include_directories(
      gazebo_ros_motor_model PUBLIC ${ignition-common3_INCLUDE_DIRS}
    )
    target_link_libraries(
      gazebo_ros_motor_model ${ignition-common3_LIBRARIES}
    )
  endif()

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()
