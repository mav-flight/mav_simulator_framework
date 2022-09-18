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
 *  @author     Suresh G
 *  @date       17.09.2022
 *  @copyright  Copyright 2022 Suresh G
 *              Licensed under the Apache License, Version 2.0 (the "License");
 *              you may not use this file except in compliance with the License.
 *              You may obtain a copy of the License at
 *
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *
 *              Unless required by applicable law or agreed to in writing, software
 *              distributed under the License is distributed on an "AS IS" BASIS,
 *              WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *              See the License for the specific language governing permissions and
 *              limitations under the License.
 */
#pragma once

// C headers
#include <gazebo/common/Plugin.hh>

namespace mav_gazebo_plugins {
// Forward declaration of private data class.
class GazeboRosMotorModelPrivate;

/// @brief  Class for custom gazebo rotor's motor model.
/// @details  Attach plugin to a gazebo MAV rotor's and publish ROS message
///           output.
/// @note Example usage:
///   @code{.xml}
///     <gazebo>
///       <plugin name="gazebo_ros_motor_model" filename="libgazebo_ros_motor_model.so">
///         <ros>
///           <namespace>demo</namespace>
///           <remapping>motor_speed:=motor_speed_demo</remapping>
///         </ros>
///
///         <publish_speed>true</publish_speed>
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
  /// @details  Gazebo calls this when the plugin is loaded.
  /// @param[in]  _model  Pointer to parent model.
  /// @param[in]  _sdf  SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  /// @brief  Update callback
  /// @details  Callback to be called at every simulation iteration.
  virtual void OnUpdate();

 private:
      ////////////////////////////////////////
      ////////////  Class Members  ///////////
      ////////////////////////////////////////

  /// Recommented PIMPL pattern.
  /// This variable should hold all private data members.
  std::unique_ptr<GazeboRosMotorModelPrivate> impl_;
};  // class GazeboRosMotorModel

}  // namespace mav_gazebo_plugins
