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

#include "quad_control_est/StateEstimateBase.hpp"

namespace quad_robot {

StateEstimateBase::StateEstimateBase(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                                     PinocchioInterface pinocchio_interface, 
                                     CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& ee_kinematics)
    : node_ptr_(node_ptr),
      pinocchio_interface_(std::move(pinocchio_interface)),
      info_(std::move(info)),
      ee_kinematics_(ee_kinematics.clone()),
      rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)) {
  
  odom_pub_ = std::make_shared<OdomPublisher>(node_ptr_->create_publisher<nav_msgs::msg::Odometry>("odom", 10));
  pose_pub_ = std::make_shared<PosePublisher>(node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10));
  last_pub_ = node_ptr_->get_clock()->now();
}

void StateEstimateBase::updateJointStates(const vector_t& joint_pos, const vector_t& joint_vel) {
  rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;
  rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, 
                                  const vector3_t& angular_vel, 
                                  const vector3_t& linear_acc,
                                  const matrix3_t& ori_cov, 
                                  const matrix3_t& angular_vel_cov,
                                  const matrix3_t& linear_acc_cov) {
  quat_ = quat;
  angular_vel_ = angular_vel;
  linear_acc_ = linear_acc;
  ori_cov_ = ori_cov;
  angular_vel_cov_ = angular_vel_cov;
  linear_acc_cov_ = linear_acc_cov;
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angular_vel) {
  rbd_state_.segment<3>(0) = zyx;
  rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angular_vel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linear_vel) {
  rbd_state_.segment<3>(3) = pos;
  rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linear_vel;
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

} // namespace quad_robot
