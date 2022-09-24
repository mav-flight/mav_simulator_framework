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
 *  @file       gazebo_ros_multirotor_model.cpp
 *  @brief      A custom gazebo plugin for MAV interface functionalities
 *              definitions.
 *  @details    Defines a custom MAV interface functionalities used by
 *              other packages in the gazebo API.
 *  @author     Fadri Furrer, ASL, ETH Zurich
 *  @author     Michael Burri, ASL, ETH Zurich
 *  @author     Mina Kamel, ASL, ETH Zurich
 *  @author     Janosch Nikolic, ASL, ETH Zurich
 *  @author     Markus Achtelik, ASL, ETH Zurich
 *  @author     Geoffrey Hunter
 *  @author     Suresh G
 *  @version    0.0.0
 *  @date       @showdate "%B %d, %Y" 2022-09-24
 *  @copyright  Apache License, Version 2.0
 */
#include "mav_gazebo_plugins/gazebo_ros_multirotor_model.h"

// Gazebo headers
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

// Conditional headers
#ifdef IGN_PROFILE_ENABLE
#include <ignition/common/Profiler.hh>
#endif

namespace mav_gazebo_plugins {
/// @brief  Class to hold private data members (PIMPL pattern)
class GazeboRosMultirotorModelPrivate {
 public:
  ///////////////////////////////////////////////////
  //////////// Constructors & Destructors ///////////
  ///////////////////////////////////////////////////

  /// @brief  Default Constructor
  GazeboRosMultirotorModelPrivate();

  /// @brief  Destructor
  ~GazeboRosMultirotorModelPrivate();

  //////////////////////////////////////
  //////////// Class Methods ///////////
  //////////////////////////////////////

  /// @brief  Update callback
  /// @details  Callback for every simulation iteration.
  /// @param[in]  _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  ////////////////////////////////////////
  ////////////  Class Members  ///////////
  ////////////////////////////////////////

  /// Pointer to parent model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to ROS node for communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Pointer to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to gazebo link.
  gazebo::physics::LinkPtr link_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;
};  // class GazeboRosMultirotorModelPrivate

///
GazeboRosMultirotorModel::GazeboRosMultirotorModel()
    : impl_(std::make_unique<GazeboRosMultirotorModelPrivate>()) {}

///
GazeboRosMultirotorModel::~GazeboRosMultirotorModel() {}

///
void GazeboRosMultirotorModel::Load(gazebo::physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf) {
  // Initialize model
  impl_->model_ = _model;

  // Initialize GazeboROS node from SDF parameters
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS& qos = impl_->ros_node_->get_qos();

  // Get link details
  if (!_sdf->HasElement("link_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                 "multirotor_model plugin missing <link_name>, "
                 "cannot proceed");
    impl_->ros_node_.reset();
    return;
  } else {
    auto link_name = _sdf->Get<std::string>("link_name");
    impl_->link_ = impl_->model_->GetLink(link_name);
    if (!impl_->link_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                   "Link [%s] does not exist. Aborting", link_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
  }

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosMultirotorModelPrivate::OnUpdate, impl_.get(),
                std::placeholders::_1));
}

///
GazeboRosMultirotorModelPrivate::GazeboRosMultirotorModelPrivate() {}

///
GazeboRosMultirotorModelPrivate::~GazeboRosMultirotorModelPrivate() {}

///
void GazeboRosMultirotorModelPrivate::OnUpdate(
    const gazebo::common::UpdateInfo& _info) {
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosMultirotorModelPrivate::OnUpdate");
#endif
  std::lock_guard<std::mutex> lock(lock_);

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  gazebo::common::Time current_time = _info.simTime;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMultirotorModel)
}  // namespace mav_gazebo_plugins