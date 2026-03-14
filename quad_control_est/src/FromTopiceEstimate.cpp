#include "quad_control_est/FromTopiceEstimate.hpp"

namespace quad_robot {

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

} // namespace quad_robot
