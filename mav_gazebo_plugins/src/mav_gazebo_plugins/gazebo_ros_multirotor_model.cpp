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
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Mav-Gazebo headers
#include "mav_gazebo_plugins/gazebo_ros_constants.h"

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

  /// Joint state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  /// Joints being tracked.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Whether to publish joint state messages.
  bool publish_states_;

  /// Simulation physics slowdown
  double sim_slowdown_;

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;
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

  // Get child joint details
  auto child_joints = impl_->link_->GetChildJoints();
  if (child_joints.size() == 0) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                 "multirotor_model pluging missing child joints, "
                 "cannot proceed");
    impl_->ros_node_.reset();
    return;
  } else {
    for (unsigned int i = 0; i < child_joints.size(); ++i) {
      auto child_joint = child_joints[i];
      if (!child_joint) {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Joint [%s] does not exist. Aborting",
                     child_joint->GetName().c_str());
      } else {
        impl_->joints_.push_back(child_joint);
      }
    }
  }

  // Get update rate
  auto update_rate = _sdf->Get<double>("update_rate", kDefaultUpdateRate).first;
  impl_->update_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.0;
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  // Get constants
  impl_->sim_slowdown_ =
      _sdf->Get<double>("sim_slowdown", kDefaultSimSlowdown).first;

  // Advertise the motor's angular velocity
  impl_->publish_states_ = _sdf->Get<bool>("publish_states", true).first;
  if (impl_->publish_states_) {
    impl_->joint_state_pub_ =
        (impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states",
            qos.get_publisher_qos("joint_states", rclcpp::QoS(1000))));

    RCLCPP_INFO(impl_->ros_node_->get_logger(),
                "Advertise joint states on [%s]",
                impl_->joint_state_pub_->get_topic_name());
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

  gazebo::common::Time current_time = _info.simTime;
  double sampling_time = (current_time - last_update_time_).Double();
  // Check period
  if (sampling_time < update_period_) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // Populate message
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp =
      gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  joint_state_msg.name.resize(joints_.size());
  joint_state_msg.position.resize(joints_.size());
  joint_state_msg.velocity.resize(joints_.size());

  for (unsigned int i = 0; i < joints_.size(); ++i) {
    auto joint = joints_[i];
    joint_state_msg.name[i] = joint->GetName();
    joint_state_msg.position[i] = joint->Position(0);
    joint_state_msg.velocity[i] = joint->GetVelocity(0) * sim_slowdown_;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  if (publish_states_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish joint_states");
#endif
    // Publish
    joint_state_pub_->publish(joint_state_msg);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }
  // Update time
  last_update_time_ = current_time;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMultirotorModel)
}  // namespace mav_gazebo_plugins
