/******************************************************************************
Copyright (c) 2026, Yuxin Li. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS 2 Core & Lifecycle
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ROS 2 Messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Realtime Tools
#include <realtime_tools/realtime_publisher.hpp>

// OCS2 & Pinocchio
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace quad_control {

using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase {
 public:
  StateEstimateBase(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                    PinocchioInterface pinocchio_interface, 
                    CentroidalModelInfo info, 
                    const PinocchioEndEffectorKinematics& ee_kinematics);
  
  virtual ~StateEstimateBase() = default;
  
  virtual void updateJointStates(const vector_t& joint_pos, const vector_t& joint_vel);
  
  virtual void updateContact(contact_flag_t contact_flag) { contact_flag_ = std::move(contact_flag); }
  
  virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, 
                         const vector3_t& angular_vel, 
                         const vector3_t& linear_acc,
                         const matrix3_t& ori_cov, 
                         const matrix3_t& angular_vel_cov,
                         const matrix3_t& linear_acc_cov);
  
  virtual vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
  
  size_t getMode() { return stanceLeg2ModeNumber(contact_flag_); }
  
 protected:
  void updateAngular(const vector3_t& zyx, const vector_t& angular_vel);
  void updateLinear(const vector_t& pos, const vector_t& linear_vel);
  void publishMsgs(const nav_msgs::msg::Odometry& odom);
  
  // ROS 2 Lifecycle Node Handle
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;
  
  // OCS2 and Kinematics
  PinocchioInterface pinocchio_interface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
  
  // Internal States
  vector3_t zyx_offset_ = vector3_t::Zero();
  vector_t rbd_state_;
  contact_flag_t contact_flag_{};
  Eigen::Quaternion<scalar_t> quat_ = Eigen::Quaternion<scalar_t>::Identity();
  vector3_t angular_vel_ = vector3_t::Zero();
  vector3_t linear_acc_ = vector3_t::Zero();
  matrix3_t ori_cov_ = matrix3_t::Zero();
  matrix3_t angular_vel_cov_ = matrix3_t::Zero();
  matrix3_t linear_acc_cov_ = matrix3_t::Zero();
  
  // ROS 2 Realtime Publishers
  using OdomPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using PosePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>;
  
  std::shared_ptr<OdomPublisher> odom_pub_;
  std::shared_ptr<PosePublisher> pose_pub_;
  const scalar_t publish_rate_ = 200.0;
  rclcpp::Time last_pub_;
};

template <typename T>
inline T square(T a) { return a * a; }

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;
  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), static_cast<SCALAR_T>(.99999));
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

} // namespace quad_control
