/*!
 *  @file       gazebo_ros_motor_model.cpp
 *  @brief      A custom gazebo plugin for MAV motor's common functionalities
 *              definitions.
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
#include "mav_gazebo_plugins/gazebo_ros_motor_model.h"

// C headers
#include <gazebo_ros/node.hpp>

namespace mav_gazebo_plugins {
/// @brief  Class to hold private date members (PIMPL patter)
class GazeboRosMotorModelPrivate {
 public:
      ////////////////////////////////////////
      ////////////  Class Members  ///////////
      ////////////////////////////////////////

  /// Pointer to parent model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to ROS node for communication.
  gazebo_ros::Node::SharedPtr ros_node_;
};  // class GazeboRosMotorModelPrivate

///
GazeboRosMotorModel::GazeboRosMotorModel()
    : impl_(std::make_unique<GazeboRosMotorModelPrivate>()) {
}

///
GazeboRosMotorModel::~GazeboRosMotorModel() {
}

///
void GazeboRosMotorModel::Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf) {
  // Initialize model
  impl_->model_ = _model;

  // Initialize GazeboROS node from SDF parameters
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
}

///
void GazeboRosMotorModel::OnUpdate() {
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotorModel)
}  // namespace mav_gazebo_plugins