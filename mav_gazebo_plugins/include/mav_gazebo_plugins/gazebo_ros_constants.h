// Copyright 2022 Suresh G
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 *  @file       gazebo_ros_constants.h
 *  @brief      A custom gazebo common constants declarations.
 *  @details    Defines a custom MAV motor's common constants used by
 *              other packages in the gazebo API.
 *  @author     Fadri Furrer, ASL, ETH Zurich
 *  @author     Michael Burri, ASL, ETH Zurich
 *  @author     Mina Kamel, ASL, ETH Zurich
 *  @author     Janosch Nikolic, ASL, ETH Zurich
 *  @author     Markus Achtelik, ASL, ETH Zurich
 *  @author     Geoffrey Hunter
 *  @author     Suresh G
 *  @version    0.0.0
 *  @date       @showdate "%B %d, %Y" 2022-09-25
 *  @copyright  Apache License, Version 2.0
 */
#pragma once

namespace mav_gazebo_plugins {
/// Time constant used when accelerating
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;

/// Time constant used when decelerating
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;

/// Time constant used to slowdown simulator
static constexpr double kDefaultSimSlowdown = 10.0;

/// Thrust's constant for motor
static constexpr double kDefaultThrustConstant = 8.54858e-06;

/// Torque's constant for motor
static constexpr double kDefaultMomentConstant = 0.016;

/// Rotor drag's constant for motor
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;

/// Rolling moment's constant for motor
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;

/// Maximum allowed angular speed control input
static constexpr double kDefaultMaxAngularSpeed = 838.0;

/// Update rate in hz.
static constexpr double kDefaultUpdateRate = 100.0;
}  // namespace mav_gazebo_plugins
