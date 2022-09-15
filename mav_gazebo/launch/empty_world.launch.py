#!/usr/bin/env python3
""" Launch gazebo server and client with command line arguments."""

##
# @file empty_world.launch.py
#
# @brief Defines a launch description for gazebo server and client.
#
# @section description_empty_world Description
# Generates a custom launch description for gazebo server and client with
# command line arguments.
#
# @section notes_empty_world Notes
# None
#
# @section todo_empty_world TODO
# None
#
# @section author_empty_world Author(s)
# - Created  by Jose Luis Rivero
# - Created  by Louise Poubel
# - Modified by Suresh G
#
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

# Imports
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration

ARGUMENTS = [
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
                          description="Whether to display the gzserver/gzclient logs"),
    DeclareLaunchArgument(name="pause",
                          default_value="false",
                          choices=["true", "false"],
                          description="Whether to start the gzserver in a paused state"),
]

def generate_launch_description():
    """ Generate custom launch description.

    @return
        An instance of LaunchDescription class.
    """
    # Launch files
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch",
            "gzserver.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": LaunchConfiguration("verbose"),
            "pause": LaunchConfiguration("pause")
        }.items())
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch",
            "gzclient.launch.py")),
        condition=IfCondition(LaunchConfiguration("gui")),
        launch_arguments={
            "verbose": LaunchConfiguration("verbose")
        }.items())

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)

    # Add nodes to LaunchDescription
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)

    return ld
