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
 *  @brief      A custom gazebo plugin for MAV controller interface
 *              functionalities definitions.
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
#include "mav_gazebo_plugins/gazebo_ros_multirotor_controller.h"

// Gazebo headers
#include <gazebo/common/Event.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// MAV-Gazebo headers
#include "mav_gazebo_plugins/gazebo_ros_constants.h"
#include "mav_msgs/msg/actuators.hpp"

// Conditional headers
#ifdef IGN_PROFILE_ENABLE
#include <ignition/common/Profiler.hh>
#endif

namespace mav_gazebo_plugins {
/// @brief  Class to hold private data members (PIMPL pattern)
class GazeboRosMultirotorControllerPrivate {
 public:
  //////////////////////////////////////
  //////////// Class Methods ///////////
  //////////////////////////////////////

  /// @brief  Update callback
  /// @details  Callback for every simulation iteration.
  /// @param[in]  _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  /// @brief  Actuators input callback
  /// @details  Callback for every actuators control input.
  /// @param[in]  _msg  Actuators command message.
  void OnActuatorsInput(const mav_msgs::msg::Actuators::SharedPtr _msg);

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

  /// Multirotor's actuators control input subscriber.
  rclcpp::Subscription<mav_msgs::msg::Actuators>::SharedPtr
      actuators_input_sub_;

  /// Motor control input publishers being tracked.
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
      motor_ctrl_publishers_;

  /// Motor control input being tracked.
  std::vector<double> ref_motor_ctrls_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;
};  // class GazeboRosMultirotorControllerPrivate

///
GazeboRosMultirotorController::GazeboRosMultirotorController()
    : impl_(std::make_unique<GazeboRosMultirotorControllerPrivate>()) {}

///
GazeboRosMultirotorController::~GazeboRosMultirotorController() = default;

///
void GazeboRosMultirotorController::Load(gazebo::physics::ModelPtr _model,
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
                 "multirotor_controller plugin missing <link_name>, "
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

  // Advertise the motor's control input
  auto child_links = impl_->link_->GetChildJointsLinks();
  if (child_links.size() == 0) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                 "multirotor_controller pluggin missing rotor joints, "
                 "cannot proceed");
    impl_->ros_node_.reset();
    return;
  } else {
    for (unsigned int i = 0; i < child_links.size(); ++i) {
      auto child_link = child_links[i];
      if (!child_link) {
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "Invalid link. Skipping");
      } else if (child_link->GetName().find("rotor_") == std::string::npos) {
        // skip non-rotor links
        continue;
      } else {
        impl_->motor_ctrl_publishers_.push_back(
            impl_->ros_node_->create_publisher<std_msgs::msg::Float64>(
                "rotor_" +
                    std::to_string(impl_->motor_ctrl_publishers_.size()) + "/" +
                    kDefaultControlInputPubTopic,
                /*queue_size*/ 1));
      }
    }
  }

  // Initialize motor controls
  impl_->ref_motor_ctrls_.resize(impl_->motor_ctrl_publishers_.size());
  std::fill(impl_->ref_motor_ctrls_.begin(), impl_->ref_motor_ctrls_.end(),
            0.0);

  // Get update rate
  auto update_rate = _sdf->Get<double>("update_rate", kDefaultUpdateRate).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),
                 "multirotor_controller plugin missing <update_rate>, "
                 "defaults to 0.0 (as fast as possible).");
  }
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  // Subscribe the actuators control input
  impl_->actuators_input_sub_ =
      impl_->ros_node_->create_subscription<mav_msgs::msg::Actuators>(
          kDefaultActuatorsInputSubTopic,
          qos.get_subscription_qos(kDefaultActuatorsInputSubTopic,
                                   rclcpp::QoS(1)),
          std::bind(&GazeboRosMultirotorControllerPrivate::OnActuatorsInput,
                    impl_.get(), std::placeholders::_1));

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosMultirotorControllerPrivate::OnUpdate, impl_.get(),
                std::placeholders::_1));
}

///
void GazeboRosMultirotorControllerPrivate::OnUpdate(
    const gazebo::common::UpdateInfo& _info) {
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosMultirotorControllerPrivate::OnUpdate");
#endif
  std::lock_guard<std::mutex> lock(lock_);

  gazebo::common::Time current_time = _info.simTime;
  double sampling_time = (current_time - last_update_time_).Double();
  // Check period
  if (sampling_time < update_period_) {
    return;
  }
#ifdef IGN_PROFILE_ENABLE
  IGN_PROFILE_BEGIN("publish control_input");
#endif
  // Publish
  if (motor_ctrl_publishers_.size() == ref_motor_ctrls_.size()) {
    std_msgs::msg::Float64 ctrl_input_msg;
    for (unsigned int i = 0; i < ref_motor_ctrls_.size(); ++i) {
      ctrl_input_msg.data = ref_motor_ctrls_[i];
      motor_ctrl_publishers_[i]->publish(ctrl_input_msg);
    }
  } else {
    RCLCPP_WARN(ros_node_->get_logger(),
                "Invalid actuators input size [%d]. Not Publishing",
                ref_motor_ctrls_.size());
  }
#ifdef IGN_PROFILE_ENABLE
  IGN_PROFILE_END();
#endif
  // Update time
  last_update_time_ = current_time;
}

///
void GazeboRosMultirotorControllerPrivate::OnActuatorsInput(
    const mav_msgs::msg::Actuators::SharedPtr _msg) {
  std::lock_guard<std::mutex> lock(lock_);

  if (motor_ctrl_publishers_.size() == _msg->motor_speeds.size()) {
    // latest commands
    for (unsigned int i = 0; i < _msg->motor_speeds.size(); ++i) {
      ref_motor_ctrls_[i] = static_cast<double>(_msg->motor_speeds[i]);
    }
  } else {
    // republish the last received commands
    RCLCPP_WARN(ros_node_->get_logger(),
                "Invalid actuators input received. Skipping");
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMultirotorController)
}  // namespace mav_gazebo_plugins