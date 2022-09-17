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
#include <gazebo/physics/physics.hh>

namespace gazebo {

class GazeboRosMotorModel : public ModelPlugin {
 public:
      ///////////////////////////////////////////////////
      //////////// Constructors & Destructors ///////////
      ///////////////////////////////////////////////////

  /// @brief  Default Constructor
  GazeboRosMotorModel();

  /// @brief  Destructor
  virtual ~GazeboRosMotorModel();

  /// @brief  Load the plugin
  /// @param[in]  _parent
  /// @param[in]  _sdf
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
};  // class GazeboRosMotorModel

}  // namespace gazebo
