#pragma once

#include "quad_control_est/StateEstimateBase.hpp"

#include <realtime_tools/realtime_buffer.hpp>

namespace quad_robot {

class FromTopicStateEstimate : public StateEstimateBase {
 public:
  FromTopicStateEstimate(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                         PinocchioInterface pinocchio_interface, 
                         CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& ee_kinematics);
  
  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
};

} // namespace quad_robot
