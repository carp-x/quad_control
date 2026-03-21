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

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "quad_control_mpc/common/Types.h"
#include "quad_control_mpc/common/ModelSettings.h"
#include "quad_control_mpc/gait/MotionPhaseDefinition.h"

namespace quad_control {

using namespace ocs2;
using namespace quad_robot;

class StateEstimateBase {
 public:
  StateEstimateBase(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                    PinocchioInterface pinocchio_interface, 
                    CentroidalModelInfo info, 
                    const PinocchioEndEffectorKinematics& ee_kinematics);
  
  virtual ~StateEstimateBase() = default;

  virtual vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
  
  void updateJointStates(const vector_t& joint_pos, const vector_t& joint_vel);
  
  void updateContact(contact_flag_t contact_flag) { contact_flag_ = std::move(contact_flag); }
  
  void updateImu(const Eigen::Quaternion<scalar_t>& global_quat,
                 const vector3_t& imu_angular_vel_i,
                 const vector3_t& imu_linear_acc_i,
                 const matrix3_t& imu_ori_cov_i,
                 const matrix3_t& imu_angular_vel_cov_i,
                 const matrix3_t& imu_linear_acc_cov_i);
  
  size_t getMode() { return stanceLeg2ModeNumber(contact_flag_); }
  
 protected:
  void updateAngular(const vector3_t& global_rpy_b, const vector_t& global_angular_vel_b);
  void updateLinear(const vector_t& global_xyz_b, const vector_t& global_linear_vel_b);
  void publishMsgs(const nav_msgs::msg::Odometry& odom);
  
  // ROS 2 Lifecycle Node Handle
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;
  
  // OCS2 and Kinematics
  PinocchioInterface pinocchio_interface_;
  CentroidalModelInfo cm_info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
  
  // Internal States
  const vector3_t base_rpy_i_{0.00, 0.00, 0.00};
  const vector3_t base_xyz_i_{0.00, 0.00, 0.00};
  Eigen::Quaternion<scalar_t> base_quat_i_ = Eigen::Quaternion<scalar_t>::Identity();
  vector_t rbd_state_;
  contact_flag_t contact_flag_{};
  Eigen::Quaternion<scalar_t> global_quat_b_ = Eigen::Quaternion<scalar_t>::Identity();
  Eigen::Quaternion<scalar_t> global_quat_i_ = Eigen::Quaternion<scalar_t>::Identity();
  vector3_t imu_angular_vel_i_ = vector3_t::Zero();
  vector3_t imu_linear_acc_i_ = vector3_t::Zero();
  matrix3_t imu_ori_cov_i_ = matrix3_t::Zero();
  matrix3_t imu_angular_vel_cov_i_ = matrix3_t::Zero();
  matrix3_t imu_linear_acc_cov_i_ = matrix3_t::Zero();
  
  // ROS 2 Realtime Publishers
  using OdomPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using PosePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>;
  
  std::shared_ptr<OdomPublisher> odom_pub_;
  std::shared_ptr<PosePublisher> pose_pub_;
  const scalar_t publish_rate_ = 500.0;
  rclcpp::Time last_pub_;
};

template <typename T>
inline T square(T a) { return a * a; }

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToRpy(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> rpy;
  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), static_cast<SCALAR_T>(.99999));
  rpy(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  rpy(1) = std::asin(as);
  rpy(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return rpy;
}

} // namespace quad_control
