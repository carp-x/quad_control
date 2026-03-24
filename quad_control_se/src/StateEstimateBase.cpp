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

#include "quad_control_se/StateEstimateBase.hpp"

namespace quad_control {

StateEstimateBase::StateEstimateBase(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                                     PinocchioInterface pinocchio_interface, 
                                     CentroidalModelInfo cm_info,
                                     const PinocchioEndEffectorKinematics& ee_kinematics)
    : node_ptr_(node_ptr),
      pinocchio_interface_(std::move(pinocchio_interface)),
      cm_info_(std::move(cm_info)),
      ee_kinematics_(ee_kinematics.clone()),
      rbd_state_(vector_t::Zero(2 * cm_info_.generalizedCoordinatesNum)) {

  base_quat_i_ = Eigen::Quaternion<scalar_t>(
    Eigen::AngleAxis<scalar_t>(base_zyx_i_[0], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<scalar_t>(base_zyx_i_[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxis<scalar_t>(base_zyx_i_[2], Eigen::Vector3d::UnitX())
  );

  odom_pub_ = std::make_shared<OdomPublisher>(node_ptr_->create_publisher<nav_msgs::msg::Odometry>("odom", 10));
  pose_pub_ = std::make_shared<PosePublisher>(node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10));
  last_pub_ = node_ptr_->get_clock()->now();
}

void StateEstimateBase::updateJointStates(const vector_t& joint_pos, const vector_t& joint_vel) {
  rbd_state_.segment(6, cm_info_.actuatedDofNum) = joint_pos;
  rbd_state_.segment(6 + cm_info_.generalizedCoordinatesNum, cm_info_.actuatedDofNum) = joint_vel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& global_quat,
                                  const vector3_t& imu_angular_vel_i,
                                  const vector3_t& imu_linear_acc_i,
                                  const matrix3_t& imu_ori_cov_i,
                                  const matrix3_t& imu_angular_vel_cov_i,
                                  const matrix3_t& imu_linear_acc_cov_i) {
  global_quat_b_ = global_quat;
  imu_angular_vel_i_ = imu_angular_vel_i;
  imu_linear_acc_i_ = imu_linear_acc_i;
  imu_ori_cov_i_ = imu_ori_cov_i;
  imu_angular_vel_cov_i_ = imu_angular_vel_cov_i;
  imu_linear_acc_cov_i_ = imu_linear_acc_cov_i;

  global_quat_i_ = global_quat_b_ * base_quat_i_;
  global_quat_i_.normalize();
}

void StateEstimateBase::updateAngular(const vector3_t& global_zyx_b, const vector_t& global_angular_vel_b) {
  rbd_state_.segment<3>(0) = global_zyx_b;
  rbd_state_.segment<3>(cm_info_.generalizedCoordinatesNum) = global_angular_vel_b;
}

void StateEstimateBase::updateLinear(const vector_t& global_pos_b, const vector_t& global_linear_vel_b) {
  rbd_state_.segment<3>(3) = global_pos_b;
  rbd_state_.segment<3>(cm_info_.generalizedCoordinatesNum + 3) = global_linear_vel_b;
}

void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry& odom) {
  rclcpp::Time time = odom.header.stamp;
  
  if (time - last_pub_ >= rclcpp::Duration::from_seconds(1.0 / publish_rate_)) {
    last_pub_ = time;
    if (odom_pub_->trylock()) {
      odom_pub_->msg_ = odom;
      odom_pub_->unlockAndPublish();
    }
    if (pose_pub_->trylock()) {
      pose_pub_->msg_.header = odom.header;
      pose_pub_->msg_.pose = odom.pose;
      pose_pub_->unlockAndPublish();
    }
  }
}

} // namespace quad_control
