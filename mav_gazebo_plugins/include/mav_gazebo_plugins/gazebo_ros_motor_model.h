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
 *  @file       gazebo_ros_motor_model.h
 *  @brief      A custom gazebo plugin for MAV motor's common functionalities
 *              declarations.
 *  @details    Defines a custom MAV motor's functionalities used by
 *              other packages in the gazebo API.
 *  @author     Fadri Furrer, ASL, ETH Zurich
 *  @author     Michael Burri, ASL, ETH Zurich
 *  @author     Mina Kamel, ASL, ETH Zurich
 *  @author     Janosch Nikolic, ASL, ETH Zurich
 *  @author     Markus Achtelik, ASL, ETH Zurich
 *  @author     Geoffrey Hunter
 *  @author     Suresh G
 *  @version    0.0.0
 *  @date       @showdate "%B %d, %Y" 2022-09-17
 *  @copyright  Apache License, Version 2.0
 */
#pragma once

// C++ headers
#include <memory>

// Gazebo headers
#include <gazebo/common/Plugin.hh>

namespace mav_gazebo_plugins {
/// @enum MotorControlType
/// @brief  Enum to hold types of motor control
/// @details
enum class MotorControlType {
  kAngularSpeed = 0,
};

/// @brief  Return the signum (-1, 0, 1) of the input value.
/// @param[in]  val The input value.
/// @return The sign of the input value.
/// @tparam T Any C/C++ data type.
template <typename T>
inline int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

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

// Forward declaration of private data class.
/// @class  GazeboRosMotorModelPrivate
class GazeboRosMotorModelPrivate;

/// @class  GazeboRosMotorModel
/// @brief  Class for custom gazebo rotor's motor model.
/// @details  Attach plugin to a gazebo MAV rotor's and publish ROS message
///           output.
/// @note  Example usage:
///   @code{.xml}
///     <gazebo>
///       <plugin name="gazebo_ros_motor_model"
///           filename="libgazebo_ros_motor_model.so">
///         <ros>
///           <namespace>demo</namespace>
///           <remapping>angular_velocity:=angular_velocity_demo</remapping>
///           <remapping>control_input:=control_input_demo</remapping>
///         </ros>
///
///         <joint_name>demo_joint</joint_name>
///         <link_name>demo_link/link_name>
///         <rotation_direction>ccw</rotation_direction>
///         <control_type>angular_speed</control_type>
///
///         <publish_velocity>true</publish_velocity>
///         <update_rate>100.0</update_rate>
///       </plugin>
///     </gazebo>
///   @endcode
///
class GazeboRosMotorModel : public gazebo::ModelPlugin {
 public:
  ///////////////////////////////////////////////////
  //////////// Constructors & Destructors ///////////
  ///////////////////////////////////////////////////

  /// @brief  Default Constructor
  GazeboRosMotorModel();

  /// @brief  Destructor
  virtual ~GazeboRosMotorModel();

 protected:
  //////////////////////////////////////
  //////////// Class Methods ///////////
  //////////////////////////////////////

  /// @brief  Load the plugin
  /// @details  Gazebo calls this method when the plugin is loaded.
  /// @param[in]  _model  Pointer to parent model.
  /// @param[in]  _sdf  SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

 private:
  ////////////////////////////////////////
  ////////////  Class Members  ///////////
  ////////////////////////////////////////

  /// Recommended PIMPL pattern.
  /// This variable should hold all private data members.
  std::unique_ptr<GazeboRosMotorModelPrivate> impl_;
};  // class GazeboRosMotorModel
}  // namespace mav_gazebo_plugins
