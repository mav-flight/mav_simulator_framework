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
 *  @file       gazebo_ros_multirotor_controller.h
 *  @brief      A custom gazebo plugin for MAV controller interface
 *              functionalities declarations.
 *  @details    Defines a custom MAV controller interface functionalities
 *              used by other packages in the gazebo API.
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

// C++ headers
#include <memory>

// Gazebo headers
#include <gazebo/common/Plugin.hh>

namespace mav_gazebo_plugins {
// Forward declaration of private data class.
/// @class  GazeboRosMultirotorControllerPrivate
class GazeboRosMultirotorControllerPrivate;

/// Control input topic publisher
static const std::string kDefaultControlInputPubTopic = "control_input";

/// Actuators control input topic subscriber
static const std::string kDefaultActuatorsInputSubTopic = "actuators_input";

/// @class  GazeboRosMultirotorController
/// @brief  Class for custom gazebo MAV controller interface.
/// @details  Attach plugin to a gazebo MAV and publish ROS message output.
/// @note  Example usage:
///   @code{.xml}
///   <gazebo>
///   <plugin name="gazebo_ros_multirotor_controller"
///     filename="libgazebo_ros_multirotor_controller.so">
///     <ros>
///       <!-- Add a namespace -->
///       <namespace>demo</namespace>
///
///       <!-- Remap the actuator input topic -->
///       <remapping>actuators_input:=actuators_input_demo</remapping>
///     </ros>
///
///     <!-- Name of the link within this model whose actuators will be commanded -->
///     <link_name>demo_link</link_name>
///
///     <!-- Update rate in Hz, defaults to 0.0, which means as fast as possible -->
///     <update_rate>100.0</update_rate>
///   </plugin>
///   </gazebo>
///   @endcode
class GazeboRosMultirotorController : public gazebo::ModelPlugin {
 public:
  ///////////////////////////////////////////////////
  //////////// Constructors & Destructors ///////////
  ///////////////////////////////////////////////////

  /// @brief  Default Constructor
  GazeboRosMultirotorController();

  /// @brief  Destructor
  virtual ~GazeboRosMultirotorController();

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
  std::unique_ptr<GazeboRosMultirotorControllerPrivate> impl_;
};  // class GazeboRosMultirotorController
}  // namespace mav_gazebo_plugins
