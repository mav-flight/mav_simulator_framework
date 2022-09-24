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
 *  @file       gazebo_ros_common.h
 *  @brief      A custom gazebo common functionalities declarations.
 *  @details    Defines a custom MAV motor's common functionalities used by
 *              other packages in the gazebo API.
 *  @author     Fadri Furrer, ASL, ETH Zurich
 *  @author     Michael Burri, ASL, ETH Zurich
 *  @author     Mina Kamel, ASL, ETH Zurich
 *  @author     Janosch Nikolic, ASL, ETH Zurich
 *  @author     Markus Achtelik, ASL, ETH Zurich
 *  @author     Geoffrey Hunter
 *  @author     Suresh G
 *  @date       @showdate "%B %d, %Y" 2022-9-18
 */
#pragma once

namespace mav_gazebo_plugins {
/// @class  FirstOrderFilter
/// @brief  Class to apply a first oder filter on a signal.
/// @details  ZoH(Zero Order Hold) discrete-time approximation to
///           first order continuous-time linear system.
///           \f[
///               \tau \frac{dx(t)}{dt} + x(t) = u(t)
///           \f]
///           where, \f$x(t)\f$ denote a generic output,
///           \f$u(t)\f$ denote a generic input and
///           \f$\tau\f$ denote a time constant [s].
/// @note For detail explanation of derivation
/// https://dsp.stackexchange.com/questions/71919/first-order-hold-discrete-time-approximation-to-first-order-continuous-time-line
/// @tparam T Any C/C++ data type.
template <class T>
class FirstOrderFilter {
 public:
  ///////////////////////////////////////////////////
  //////////// Constructors & Destructors ///////////
  ///////////////////////////////////////////////////

  /// @brief  Constructor
  /// @param[in]  time_const_up
  /// @param[in]  time_const_down
  /// @param[in]  initial_state
  FirstOrderFilter(double time_const_up, double time_const_down,
                   T initial_state)
      : time_const_up_(time_const_up),
        time_const_down_(time_const_down),
        previous_state_(initial_state) {}

  /// @brief  Destructor
  ~FirstOrderFilter() {}

  /// @brief  Apply a first order filter on the input state.
  /// @param[in]  input_state
  /// @param[in]  sampling_time
  /// @return
  T UpdateFilter(T input_state, double sampling_time) {
    double alpha;
    if (input_state > previous_state_) {
      // Acceleration
      alpha = std::exp(-sampling_time / time_const_up_);
    } else {
      // Deceleration
      alpha = std::exp(-sampling_time / time_const_down_);
    }

    // x(k+1) = Ad * x(k) + Bd * u(k)
    T output_state = alpha * previous_state_ + (1.0 - alpha) * input_state;
    previous_state_ = output_state;

    return output_state;
  }

 protected:
  ////////////////////////////////////////
  ////////////  Class Members  ///////////
  ////////////////////////////////////////

  /// ODE time constant when accelerating.
  double time_const_up_;

  /// ODE time constant when decelerating.
  double time_const_down_;

  /// ODE's system state.
  T previous_state_;
};  // class FirstOrderFilter
}  // namespace mav_gazebo_plugins
