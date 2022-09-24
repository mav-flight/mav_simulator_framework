#!/usr/bin/env python3
""" Spawn MAV model with command line arguments. """

# Copyright 2022 Suresh G
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

##
# @file       spawn_mav.launch.py
# @brief      Defines a launch description for spawing MAV.
# @details    Generates a custom launch description for spawing mav model.
# @author     Fadri Furrer, ASL, ETH Zurich
# @author     Michael Burri, ASL, ETH Zurich
# @author     Mina Kamel, ASL, ETH Zurich
# @author     Janosch Nikolic, ASL, ETH Zurich
# @author     Markus Achtelik, ASL, ETH Zuich
# @author     Surech G
# @date       @showdate "%B %d, %Y" 2022-09-18
# @copyright  Apache License, Version 2.0

# Python Imports
import os
from typing import List

# ROS Imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions.launch_configuration import LaunchConfiguration

# @cond PRIVATE
# The launch description's initial arguments
ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(name="mav_name",
                          default_value="hummingbird",
                          description="MAV model name"),
    DeclareLaunchArgument(name="mav_model",
                          default_value=os.path.join(
                              get_package_share_directory("mav_description"),
                              "urdf",
                              "hummingbird.urdf.xacro"),
                          description="Full path of MAV model's xacro file"),
    DeclareLaunchArgument(name="initial_pose_x",
                          default_value="0.0",
                          description="Initial x-position"),
    DeclareLaunchArgument(name="initial_pose_y",
                          default_value="0.0",
                          description="Initial y-position"),
    DeclareLaunchArgument(name="initial_pose_z",
                          default_value="0.1",
                          description="Initial z-position"),
    DeclareLaunchArgument(name="use_sim_time",
                          default_value="true",
                          choices=["true", "false"],
                          description="Whether to use the simulation time"),
]
# @endcond


def generate_launch_description() -> LaunchDescription:
    """ Generate custom launch description.

    Returns:
        An instance of LaunchDescription class.
    """
    # robot state publisher node
    rsp_node = Node(package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[
                        {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        {"robot_description": Command([
                            "xacro", " ",
                            LaunchConfiguration("mav_model"), " ",
                            "namespace:=", LaunchConfiguration("mav_name"),
                        ])},
                    ])

    # spawn entity node
    se_node = Node(package="gazebo_ros",
                   executable="spawn_entity.py",
                   arguments=[
                       "-entity", LaunchConfiguration("mav_name"),
                       "-topic", "robot_description",
                       "-timeout", "120",
                       "-x", LaunchConfiguration("initial_pose_x"),
                       "-y", LaunchConfiguration("initial_pose_y"),
                       "-z", LaunchConfiguration("initial_pose_z"),
                   ],
                   output="screen",
                   parameters=[
                       {"use_sim_time": LaunchConfiguration("use_sim_time")}
                   ])

    # Define LaunchDescription variable
    launch_desc = LaunchDescription(ARGUMENTS)

    # Add nodes to LaunchDescription
    launch_desc.add_action(rsp_node)
    launch_desc.add_action(se_node)

    return launch_desc
