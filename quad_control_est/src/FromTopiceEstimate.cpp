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

#include "quad_control_est/FromTopiceEstimate.hpp"

namespace quadruped_estimate {

FromTopicStateEstimate::FromTopicStateEstimate(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                                               PinocchioInterface pinocchio_interface, 
                                               CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& ee_kinematics)
    : StateEstimateBase(node_ptr, std::move(pinocchio_interface), std::move(info), ee_kinematics) {
  
  sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    "/ground_truth/state", 
    rclcpp::SensorDataQoS(), 
    std::bind(&FromTopicStateEstimate::callback, this, std::placeholders::_1));
}

void FromTopicStateEstimate::callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const auto* odom_ptr = buffer_.readFromRT();
  if (!odom_ptr) {
    return rbd_state_;
  }
  const auto& odom = *odom_ptr;

  updateAngular(quatToZyx(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
                vector3_t(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(vector3_t(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               vector3_t(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  publishMsgs(odom);

  return rbd_state_;
}

} // namespace quadruped_estimate
