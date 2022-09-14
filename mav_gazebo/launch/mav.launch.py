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

"""
     Launch gazebo with spawned mav model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Returns:
        An instance of LaunchDescription class.
    """
    # Launch files
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mav_gazebo"), "launch",
            "empty_world.launch.py")),
        launch_arguments={
            "world": "",
            "gui": "true",
            "verbose": "true",
            "pause": "true"
        }.items())
    mav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mav_gazebo"), "launch",
            "spawn_mav.launch.py")),
        launch_arguments={
            "initial_pose_x": "0.0",
            "initial_pose_y": "0.0",
            "initial_pose_z": "0.5"
        }.items())

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(gz_launch)
    ld.add_action(mav_launch)

    return ld
