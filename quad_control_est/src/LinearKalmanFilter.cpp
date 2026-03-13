#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "quad_control_est/LinearKalmanFilter.hpp"

namespace quad_robot {

LinearKalmanFilter::LinearKalmanFilter(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                                       PinocchioInterface pinocchio_interface,
                                       CentroidalModelInfo info,
                                       const PinocchioEndEffectorKinematics& ee_kinematics)
    : StateEstimateBase(node_ptr, pinocchio_interface, info, ee_kinematics),
      num_contacts_(info_.numThreeDofContacts),
      dim_contacts_(3 * num_contacts_),
      num_state_(6 + dim_contacts_),
      num_observe_(2 * dim_contacts_ + num_contacts_) {
  
  x_hat_.setZero(num_state_);
  u_.setZero(3);
  z_.setZero(num_observe_);
  A_.setIdentity(num_state_, num_state_);
  B_.setZero(num_state_, 3);
  Q_.setZero(num_state_, num_state_);
  P_.setIdentity(num_state_, num_state_) * 100.0;
  R_.setZero(num_observe_, num_observe_);
  feet_heights_.setZero(num_contacts_);
  
  C_.setZero(num_observe_, num_state_);
  for (size_t i = 0; i < num_contacts_; ++i) {
    C_.block<3, 3>(3 * i, 0) = matrix3_t::Identity();                   // 关联 base_pos
    C_.block<3, 3>(3 * (num_contacts_ + i), 3) = matrix3_t::Identity(); // 关联 base_vel
    C_(2 * dim_contacts_ + i, 6 + 3 * i + 2) = 1.0;                     // 关联 ee_pos_z
  }
  C_.block(0, 6, dim_contacts_, dim_contacts_) = -matrix_t::Identity(dim_contacts_, dim_contacts_);
}

vector_t LinearKalmanFilter::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  scalar_t dt = period.seconds();

  preparePredict(dt, A_, B_, Q_, u_);
  predictXP(x_hat_, P_, A_, B_, Q_, u_);

  prepareUpdate(z_, C_, R_);
  updateXP(x_hat_, P_, z_, C_, R_);

  updateLinear(x_hat_.head<3>(), x_hat_.segment<3>(3));
  
  auto odom = getOdomMsg();
  odom.header.stamp = time;
  publishMsgs(odom);

  return rbd_state_;
}

void LinearKalmanFilter::preparePredict(scalar_t dt, matrix_t& A, matrix_t& B, matrix_t& Q, vector_t& u) {
  // A = [I, I*dt; 0, I]
  A.block<3, 3>(0, 3) = dt * matrix3_t::Identity();
  
  // B = [0.5*dt^2; dt]
  B.block<3, 3>(0, 0) = 0.5 * dt * dt * matrix3_t::Identity();
  B.block<3, 3>(3, 0) = dt * matrix3_t::Identity();

  // Q (过程噪声/模型不确定性)
  Q.block<3, 3>(0, 0) = (dt / 20.0) * matrix3_t::Identity() * q_base_pos_;
  Q.block<3, 3>(3, 3) = (dt * 9.81 / 20.0) * matrix3_t::Identity() * q_base_vel_;
  Q.block(6, 6, dim_contacts_, dim_contacts_) = dt * matrix_t::Identity(dim_contacts_, dim_contacts_) * q_ee_pos_static_;

  // 计算世界系下的合加速度 (旋转机体加速度并补偿重力)
  u = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linear_acc_local_ + vector3_t(0, 0, -9.81);
}

void LinearKalmanFilter::predictXP(vector_t& x, matrix_t& P, const matrix_t& A, const matrix_t& B, const matrix_t& Q, const vector_t& u) {
  x = A * x + B * u;
  P = A * P * A.transpose() + Q;
}

void LinearKalmanFilter::prepareUpdate(vector_t& z, matrix_t& /*H*/, matrix_t& R) {
  // 同步 Pinocchio 运动学
  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  vector_t q_pino = vector_t::Zero(info_.generalizedCoordinatesNum);
  q_pino.segment<3>(3) = rbd_state_.head<3>(); // 仅姿态
  q_pino.tail(info_.actuatedDofNum) = rbd_state_.segment(6, info_.actuatedDofNum);
  
  pinocchio::forwardKinematics(model, data, q_pino);
  pinocchio::updateFramePlacements(model, data);

  const auto ee_pos = ee_kinematics_->getPosition(vector_t());
  const auto ee_vel = ee_kinematics_->getVelocity(vector_t(), vector_t());

  // 动态构建观测向量与噪声权重
  for (size_t i = 0; i < num_contacts_; i++) {
    scalar_t trust = contact_flag_[i] ? 1.0 : 100.0; // 摆动腿不信任度增加
    
    // z 观测: [相对位置; 相对速度; 绝对高度]
    z.segment<3>(3 * i) = -ee_pos[i];
    z.segment<3>(3 * i)(2) += foot_radius_;
    z.segment<3>(dim_contacts_ + 3 * i) = -ee_vel[i];
    z(2 * dim_contacts_ + i) = feet_heights_[i];

    // R 观测噪声 (对角线赋值)
    R.block<3, 3>(3 * i, 3 * i) = matrix3_t::Identity() * r_ee_pos_ * trust;
    R.block<3, 3>(dim_contacts_ + 3 * i, dim_contacts_ + 3 * i) = matrix3_t::Identity() * r_ee_vel_ * trust;
    R(2 * dim_contacts_ + i, 2 * dim_contacts_ + i) = r_ee_height_ * trust;
  }
}

void LinearKalmanFilter::updateXP(vector_t& x, matrix_t& P, const vector_t& z, const matrix_t& H, const matrix_t& R) {
  // S = HPH' + R
  matrix_t S = H * P * H.transpose() + R;
  // K = PH'S^-1 (LU分解求解)
  matrix_t K = P * H.transpose() * S.lu().solve(matrix_t::Identity(num_observe_, num_observe_));
  
  // 更新状态与协方差
  x += K * (z - H * x);
  P = (matrix_t::Identity(num_state_, num_state_) - K * H) * P;
  P = (P + P.transpose()) * 0.5; // 对称化
}

nav_msgs::msg::Odometry LinearKalmanFilter::getOdomMsg() {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = x_hat_(0);
  odom.pose.pose.position.y = x_hat_(1);
  odom.pose.pose.position.z = x_hat_(2);
  
  odom.pose.pose.orientation.w = quat_.w();
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  
  vector3_t linear_vel_base = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * x_hat_.segment<3>(3);
  odom.twist.twist.linear.x = linear_vel_base.x();
  odom.twist.twist.linear.y = linear_vel_base.y();
  odom.twist.twist.linear.z = linear_vel_base.z();
  
  return odom;
}

} // namespace quad_robot
