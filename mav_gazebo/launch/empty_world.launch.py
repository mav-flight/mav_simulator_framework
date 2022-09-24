#!/usr/bin/env python3
""" Launch gazebo server and client with command line arguments. """

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
# @file       empty_world.launch.py
# @brief      Defines a launch description for gazebo server and client.
# @details    Generates a custom launch description for gazebo server and client
#             with command line arguments.
# @author     Jose Luis Rivero, Open Source Robotics Foundation, Inc.
# @author     Louise Poubel, Open Source Robotics Foundation, Inc.
# @author     Suresh G
# @date       @showdate "%B %d, %Y" 2022-09-18
# @copyright  Apache License, Version 2.0

# Python Imports
import os
from typing import List

# ROS Imports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration

# @cond PRIVATE
# The launch description's initial arguments
ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(name="world",
                          default_value="",
                          description="Specify the world file name"),
    DeclareLaunchArgument(name="gui",
                          default_value="true",
                          choices=["true", "false"],
                          description="Whether to run as headless"),
    DeclareLaunchArgument(name="verbose",
                          default_value="false",
                          choices=["true", "false"],
                          description="Whether to display gzserver and "
                                      "gzclient logs"),
    DeclareLaunchArgument(name="pause",
                          default_value="false",
                          choices=["true", "false"],
                          description="Whether to start gzserver in a "
                                      "paused state"),
]
# @endcond


def generate_launch_description() -> LaunchDescription:
    """ Generate custom launch description.

    Returns
        An instance of LaunchDescription class.
    """
    # gazebo server
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch",
                         "gzserver.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": LaunchConfiguration("verbose"),
            "pause": LaunchConfiguration("pause")
        }.items())

    # gazebo client
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch",
                         "gzclient.launch.py")),
        condition=IfCondition(LaunchConfiguration("gui")),
        launch_arguments={
            "verbose": LaunchConfiguration("verbose")
        }.items())

    # Define LaunchDescription variable
    launch_desc = LaunchDescription(ARGUMENTS)

    # Add nodes to LaunchDescription
    launch_desc.add_action(gzserver_launch)
    launch_desc.add_action(gzclient_launch)

    return launch_desc
