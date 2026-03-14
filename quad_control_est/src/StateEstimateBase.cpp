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
  scalar_t publish_rate = 200;
  
  if (time - last_pub_ >= rclcpp::Duration::from_seconds(1.0 / publish_rate)) {
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
