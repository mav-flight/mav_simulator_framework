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
 *  @author     Geoffrey Hunter
 *  @author     Suresh G
 *  @version    0.0.0
 *  @date       @showdate "%B %d, %Y" 2022-09-17
 *  @copyright  Apache License, Version 2.0
 */
#include "mav_gazebo_plugins/gazebo_ros_motor_model.h"

// Gazebo headers
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// Mav-Gazebo headers
#include "mav_gazebo_plugins/gazebo_ros_common.h"
#include "mav_gazebo_plugins/gazebo_ros_constants.h"

// Conditional headers
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

namespace mav_gazebo_plugins {
/// @brief  Class to hold private date members (PIMPL pattern)
class GazeboRosMotorModelPrivate {
 public:
  ///////////////////////////////////////////////////
  //////////// Constructors & Destructors ///////////
  ///////////////////////////////////////////////////

  /// @brief  Default Constructor
  GazeboRosMotorModelPrivate();

  /// @brief  Destructor
  ~GazeboRosMotorModelPrivate();

  //////////////////////////////////////
  //////////// Class Methods ///////////
  //////////////////////////////////////

  /// @brief  Update callback
  /// @details  Callback for every simulation iteration.
  /// @param[in]  _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  /// @brief  Control input callback
  /// @details  Callback for every motor control input.
  /// @param[in]  _msg Control command message.
  void OnControlInput(const std_msgs::msg::Float64::SharedPtr _msg);

  ////////////////////////////////////////
  ////////////  Class Members  ///////////
  ////////////////////////////////////////

  /// Pointer to parent model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to ROS node for communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to gazebo joint.
  gazebo::physics::JointPtr joint_;

  /// Pointer to gazebo link.
  gazebo::physics::LinkPtr link_;

  /// Motor rotation direction.
  /// 'Counter-Clockwise' rotation means +1 and 'Clockwise' rotation means -1.
  int rotation_sign_;

  /// Motor control type.
  MotorControlType ctrl_type_;

  /// Motor control input
  double ref_ctrl_input_;

  /// Motor control input subscriber.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ctrl_input_sub_;

  /// Motor angular velocity publisher.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_velocity_pub_;

  /// First order filter for angular speed.
  std::unique_ptr<FirstOrderFilter<double>> angular_speed_filter_;

  /// Wind speed in world frame
  ignition::math::Vector3d wind_speed_W_;

  /// Motor angular velocity.
  std_msgs::msg::Float64 velocity_msg_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Whether to publish motor angular velocity messages.
  bool publish_velocity_;

  /// Update period in seconds.
  double update_period_;

  /// Seconds since last update.
  double sampling_time_;

  /// Acceleration time constant
  double time_const_up_;

  /// Deceleration time constant
  double time_const_down_;

  /// Simulation physics slowdown
  double sim_slowdown_;

  /// Thrust constant
  double thrust_const_;

  /// Rotor drag coefficient
  double rotor_drag_coeff_;

  /// Moment constant
  double moment_const_;

  /// Rolling moment coefficient
  double rolling_moment_coeff_;

  /// Maximum allowed angular speed input.
  double max_angular_speed_;

  /// Last update time.
  gazebo::common::Time last_update_time_;
};  // class GazeboRosMotorModelPrivate

///
GazeboRosMotorModel::GazeboRosMotorModel()
    : impl_(std::make_unique<GazeboRosMotorModelPrivate>()) {}

///
GazeboRosMotorModel::~GazeboRosMotorModel() {}

///
void GazeboRosMotorModel::Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf) {
  // Initialize model
  impl_->model_ = _model;

  // Initialize GazeboROS node from SDF parameters
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS& qos = impl_->ros_node_->get_qos();

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

  // Get rotation direction
  if (!_sdf->HasElement("rotation_direction")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                 "motor_model plugin missimg <rotation_direction>, "
                 "cannot proceed");
    impl_->ros_node_.reset();
    return;
  } else {
    auto rotation_direction = _sdf->Get<std::string>("rotation_direction");
    // +1 for "ccw", -1 for "cw" and 0 otherwise
    impl_->rotation_sign_ =
        (("ccw" == rotation_direction) - ("cw" == rotation_direction));
    if (!impl_->rotation_sign_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                   "Rotation direction [%s] is not valid. "
                   "Allowed values [ccw, cw]",
                   rotation_direction.c_str());
      impl_->ros_node_.reset();
      return;
    }
  }

  // Get motor type
  if (!_sdf->HasElement("control_type")) {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
                "motor_model plugin missing <control_type>, cannot proceed");
    impl_->ros_node_.reset();
    return;
  } else {
    auto control_type = _sdf->Get<std::string>("control_type");
    if ("angular_speed" == control_type) {
      impl_->ctrl_type_ = MotorControlType::kAngularSpeed;
    } else {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                   "Motor control type [%s] is not valid. "
                   "Allowed values [angular_speed]",
                   control_type.c_str());
      impl_->ros_node_.reset();
      return;
    }
  }

  // Get update rate
  auto update_rate = _sdf->Get<double>("update_rate", kDefaultUpdateRate).first;
  impl_->update_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.0;
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  // Get constants
  impl_->time_const_up_ =
      _sdf->Get<double>("time_constant_up", kDefaultTimeConstantUp).first;
  impl_->time_const_down_ =
      _sdf->Get<double>("time_constant_down", kDefaultTimeConstantDown).first;
  impl_->sim_slowdown_ =
      _sdf->Get<double>("sim_slowdown", kDefaultSimSlowdown).first;
  impl_->thrust_const_ =
      _sdf->Get<double>("thrust_constant", kDefaultThrustConstant).first;
  impl_->rotor_drag_coeff_ =
      _sdf->Get<double>("rotor_drag_coeff", kDefaultRotorDragCoefficient).first;
  impl_->moment_const_ =
      _sdf->Get<double>("moment_constant", kDefaultMomentConstant).first;
  impl_->rolling_moment_coeff_ =
      _sdf->Get<double>("rolling_moment_coeff",
                        kDefaultRollingMomentCoefficient)
          .first;
  impl_->max_angular_speed_ =
      _sdf->Get<double>("max_angular_speed", kDefaultMaxAngularSpeed).first;

  // Subcribe the motor's control input
  switch (impl_->ctrl_type_) {
    case MotorControlType::kAngularSpeed: {
      impl_->ctrl_input_sub_ =
          impl_->ros_node_->create_subscription<std_msgs::msg::Float64>(
              kDefaultControlInputSubTopic,
              qos.get_subscription_qos(kDefaultControlInputSubTopic,
                                       rclcpp::QoS(1)),
              std::bind(&GazeboRosMotorModelPrivate::OnControlInput,
                        impl_.get(), std::placeholders::_1));
      break;
    }
    default: {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                   "Motor control input subscription is not valid.");
      impl_->ros_node_.reset();
      return;
    }
  }

  // Advertise the motor's angular velocity
  impl_->publish_velocity_ = _sdf->Get<bool>("publish_velocity", true).first;
  if (impl_->publish_velocity_) {
    impl_->angular_velocity_pub_ =
        (impl_->ros_node_->create_publisher<std_msgs::msg::Float64>(
            kDefaultAngularVelocityPubTopic,
            qos.get_publisher_qos(kDefaultAngularVelocityPubTopic,
                                  rclcpp::QoS(1))));

    RCLCPP_INFO(impl_->ros_node_->get_logger(),
                "Advertise angular velocity on [%s]",
                impl_->angular_velocity_pub_->get_topic_name());
  }

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosMotorModelPrivate::OnUpdate, impl_.get(),
                std::placeholders::_1));

  // Create the first order filter.
  impl_->angular_speed_filter_.reset(new FirstOrderFilter<double>(
      /*time_const_up*/ impl_->time_const_up_,
      /*time_const_down*/ impl_->time_const_down_,
      /*initial_state*/ impl_->ref_ctrl_input_));
}

///
GazeboRosMotorModelPrivate::GazeboRosMotorModelPrivate() {}

///
GazeboRosMotorModelPrivate::~GazeboRosMotorModelPrivate() {}

///
void GazeboRosMotorModelPrivate::OnUpdate(
    const gazebo::common::UpdateInfo& _info) {
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosMotorModelPrivate::OnUpdate");
#endif
  std::lock_guard<std::mutex> lock(lock_);

  gazebo::common::Time current_time = _info.simTime;
  sampling_time_ = (current_time - last_update_time_).Double();
  // Check period
  if (sampling_time_ < update_period_) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // Get: simulation time scaled motor's angular velocity
  double angular_velocity = joint_->GetVelocity(0) * sim_slowdown_;
  double angular_speed = std::abs(angular_velocity);
  int angular_velocity_sign = sgn(angular_velocity);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("apply thrust");
#endif
  // Compute thrust = k_h * \omega**2
  // we assume symmetric propellers here
  double thrust = (rotation_sign_ * angular_velocity_sign * thrust_const_ *
                   std::pow(angular_speed, 2));

  // Apply force to the link relative to the body frame
  link_->AddRelativeForce(ignition::math::Vector3d(0.0, 0.0, thrust));
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("apply rotor drag");
#endif
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // by Philippe Martin and Erwan Salaün
  // Compute forces H-force/rotor_drag = -1.0 * \omega * \lambda_1 * V_A^{\perp}
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
  ignition::math::Vector3d body_velocity_W =
      (link_->WorldLinearVel() - wind_speed_W_);
  ignition::math::Vector3d body_velocity_W_perp =
      (body_velocity_W - (body_velocity_W.Dot(joint_axis) * joint_axis));
  ignition::math::Vector3d rotor_drag_W =
      (-1.0 * angular_speed * rotor_drag_coeff_ * body_velocity_W_perp);

  // Apply drag to the link
  link_->AddForce(rotor_drag_W);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("apply drag torque");
#endif
  gazebo::physics::LinkPtr parent_link = link_->GetParentJointsLinks().at(0);
  // Transformation from parent_link to the link_;
  ignition::math::Pose3d pose_diff =
      (link_->WorldCoGPose() - parent_link->WorldCoGPose());
  double torque = -1.0 * moment_const_ * rotation_sign_ * thrust;
  // Transform the drag torque into the parent's frame
  ignition::math::Vector3d drag_torque_parent = (pose_diff.Rot().RotateVector(
      ignition::math::Vector3d(0.0, 0.0, torque)));

  // Apply torque to the link relative to the parent's body frame
  link_->AddRelativeTorque(drag_torque_parent);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("apply rolling moment");
#endif
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // by Philippe Martin and Erwan Salaün
  // Compute forces H-force/rotor_drag = -1.0 * \omega * \mu_1 * V_A^{\perp}
  ignition::math::Vector3d rolling_moment_W =
      (-1.0 * angular_speed * rolling_moment_coeff_ * body_velocity_W_perp);

  // Apply moment to the parent's link
  parent_link->AddTorque(rolling_moment_W);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("update joint velocity");
#endif
  // Apply the first order filter on the motor's speed.
  double new_angular_speed = angular_speed_filter_->UpdateFilter(
      /*input_state*/ ref_ctrl_input_, /*sampling_time*/ sampling_time_);

  // Set: simulation time scale motor's angular velocity
  joint_->SetVelocity(
      /*axis*/ 0,
      /*velocity*/ rotation_sign_ * new_angular_speed / sim_slowdown_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

  if (publish_velocity_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish angular_velocity");
#endif
    // Publish
    velocity_msg_.data = joint_->GetVelocity(0);
    angular_velocity_pub_->publish(velocity_msg_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }
  // Update time
  last_update_time_ = current_time;
}

///
void GazeboRosMotorModelPrivate::OnControlInput(
    const std_msgs::msg::Float64::SharedPtr _msg) {
  std::lock_guard<std::mutex> lock(lock_);

  switch (ctrl_type_) {
    case MotorControlType::kAngularSpeed: {
      ref_ctrl_input_ =
          std::min(static_cast<double>(_msg->data), max_angular_speed_);
      break;
    }
    default: {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "Motor control input is not valid.");
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotorModel)
}  // namespace mav_gazebo_plugins
