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
 *  @date       @showdate "%B %d, %Y" 2022-9-17
 */
#include "mav_gazebo_plugins/gazebo_ros_motor_model.h"

// C headers
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
  #include <ignition/common/Profiler.hh>
#endif
#include <std_msgs/msg/float32.hpp>

namespace mav_gazebo_plugins {
/// @brief  Class to hold private date members (PIMPL patter)
class GazeboRosMotorModelPrivate {
 public:
      //////////////////////////////////////
      //////////// Class Methods ///////////
      //////////////////////////////////////

  /// @brief  Update callback
  /// @details  Callback to be called at every simulation iteration.
  /// @param[in]  _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

      ////////////////////////////////////////
      ////////////  Class Members  ///////////
      ////////////////////////////////////////

  /// Pointer to parent model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to ROS node for communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Connection to event called at every world iteration,
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to gazebo joint.
  gazebo::physics::JointPtr joint_;

  /// Pointer to gazebo link.
  gazebo::physics::LinkPtr link_;

  /// Motor speed publisher.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_speed_pub_;

  /// Motor rotating speed.
  std_msgs::msg::Float32 speed_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Whether to publish motor speed messages.
  bool publish_speed_;

  /// Update period in seconds.
  double update_period_;

  /// Seconds since last update.
  double sampling_time_;

  /// Last update time.
  gazebo::common::Time last_update_time_;
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

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get joint details
  if (!_sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                 "motor_model plugin missing <joint_name>, cannot proceed");
    impl_->ros_node_.reset();
    return;
  } else {
    auto joint_name = _sdf->Get<std::string>("joint_name");
    impl_->joint_ = impl_->model_->GetJoint(joint_name);
    if (!impl_->joint_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                   "Joint [%s] does not exist. Aborting", joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
  }

  // Get link details
  if (!_sdf->HasElement("link_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                 "motor_model plugin missing <link_name>, cannot proceed");
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

  // Get update rate
  auto update_rate = _sdf->Get<double>("update_rate", 0.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0/update_rate;
  }
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  // Advertise the motor's speed
  impl_->publish_speed_ = _sdf->Get<bool>("publish_speed", true).first;
  if (impl_->publish_speed_) {
    impl_->motor_speed_pub_ = (
      impl_->ros_node_->create_publisher<std_msgs::msg::Float32>("motor_speed",
        qos.get_publisher_qos("motor_speed", rclcpp::QoS(1))));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise motor speed on [%s]",
                impl_->motor_speed_pub_->get_topic_name());
  }

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosMotorModelPrivate::OnUpdate, impl_.get(),
              std::placeholders::_1));
}

///
void GazeboRosMotorModelPrivate::OnUpdate(
    const gazebo::common::UpdateInfo& _info) {
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosMotorModelPrivate::OnUpdate");
  #endif
  std::lock_guard<std::mutex> lock(lock_);

  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("fill ROS message");
  #endif
  gazebo::common::Time current_time = _info.simTime;
  sampling_time_ = (current_time - last_update_time_).Double();

  if (sampling_time_ < update_period_) {
    return;
  }
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
  #endif

  if (publish_speed_) {
    #ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("publish motor_speed");
    #endif
    speed_.data = joint_->GetVelocity(0);
    motor_speed_pub_->publish(speed_);
    #ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
    #endif
  }

  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("update");
  #endif
  last_update_time_ = current_time;
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
  #endif
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotorModel)
}  // namespace mav_gazebo_plugins
