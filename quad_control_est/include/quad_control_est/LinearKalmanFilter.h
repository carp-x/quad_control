#pragma once

#include "quad_control_est/StateEstimateBase.hpp"

namespace quad_robot {

class LinearKalmanFilter : public StateEstimateBase {
 public:
  LinearKalmanFilter(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                     PinocchioInterface pinocchio_interface,
                     CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics& ee_kinematics);
  
  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
 protected:
  nav_msgs::msg::Odometry getOdomMsg();
  
  const scalar_t q_base_pos_      = 0.02;
  const scalar_t q_base_vel_      = 0.02;
  const scalar_t q_ee_pos_static_ = 0.002;

  const scalar_t r_ee_pos_        = 0.005;
  const scalar_t r_ee_vel_        = 0.1;
  const scalar_t r_ee_height_     = 0.01;
  
  const scalar_t foot_radius_ = 0.02;
  vector_t feet_heights_;
  
 private:
  void preparePredict(scalar_t dt, matrix_t& A, matrix_t& B, matrix_t& Q, vector_t& u);
  // x_new = Ax + Bu, P_new = APA' + Q
  void predictXP(vector_t& x, matrix_t& P, const matrix_t& A, const matrix_t& B, const matrix_t& Q, const vector_t& u);
  
  void prepareUpdate(vector_t& z, matrix_t& H, matrix_t& R);
  // x += K * (z - H * x_new)
  void updateXP(vector_t& x, matrix_t& P, const vector_t& z, const matrix_t& H, const matrix_t& R);
  
  size_t num_contacts_, dim_contacts_, num_state_, num_observe_;
  
  matrix_t A_, B_, C_, Q_, P_, R_;
  vector_t x_hat_, u_, z_;
};

} // namespace quad_robot
