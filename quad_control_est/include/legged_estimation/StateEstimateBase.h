#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS 2 Core & Lifecycle
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ROS 2 Messages
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Realtime Tools
#include <realtime_tools/realtime_publisher.hpp>

// OCS2 & Pinocchio
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace quad_robot {

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
                         const vector3_t& angular_vel_local, 
                         const vector3_t& linear_acc_local,
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
  vector3_t angular_vel_local_ = vector3_t::Zero();
  vector3_t linear_acc_local_ = vector3_t::Zero();
  matrix3_t ori_cov_ = matrix3_t::Zero();
  matrix3_t angular_vel_cov_ = matrix3_t::Zero();
  matrix3_t linear_acc_cov_ = matrix3_t::Zero();
  
  // ROS 2 Realtime Publishers
  using OdomPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using PosePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>;
  
  std::shared_ptr<OdomPublisher> odom_pub_;
  std::shared_ptr<PosePublisher> pose_pub_;
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

}  // namespace quad_robot
