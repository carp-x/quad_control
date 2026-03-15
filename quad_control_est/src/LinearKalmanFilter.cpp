#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include "quad_control_est/LinearKalmanFilter.hpp"

namespace quad_robot {

LinearKalmanFilter::LinearKalmanFilter(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                                       PinocchioInterface pinocchio_interface,
                                       CentroidalModelInfo info,
                                       const PinocchioEndEffectorKinematics& ee_kinematics)
    : StateEstimateBase(node_ptr, std::move(pinocchio_interface), std::move(info), ee_kinematics),
      num_contacts_(info_.numThreeDofContacts),
      dim_contacts_(3 * num_contacts_),
      num_state_(6 + dim_contacts_),
      num_observe_(2 * dim_contacts_ + num_contacts_) {
  
  // [base_pos(3), base_vel(3), ee_pos_1(3), ..., ee_pos_4(3)]^T 
  x_hat_.setZero(num_state_);
  // [acc_x, acc_y, acc_z]^T (World Frame)
  u_.setZero(3);
  // [ee_rel_pos(12), ee_rel_vel(12), ee_height(4)]^T
  z_.setZero(num_observe_);
  
  // A = [ I(3x3)  I*dt(3x3)  0 ]
  //     [   0       I(3x3)   0 ]
  //     [   0         0      I ]
  A_.setIdentity(num_state_, num_state_);
  
  // B = [ 0.5*dt^2 * I ]
  //     [     dt   * I ]
  //     [       0      ]
  B_.setZero(num_state_, 3);

  // [  I(3x3)   0(3x3)  -I(3x3)    0       0       0    ] -> ee_rel_pos_1
  // [    ...      ...      ...     ...     ...     ...  ]
  // [  0(3x3)   I(3x3)     0       0       0       0    ] -> ee_rel_vel_1
  // [    ...      ...      ...     ...     ...     ...  ]
  // [    0        0      [0,0,1]   0       0       0    ] -> ee_height_1
  C_.setZero(num_observe_, num_state_);
  for (size_t i = 0; i < num_contacts_; ++i) {
    C_.block<3, 3>(3 * i, 0) = matrix3_t::Identity();                   
    C_.block<3, 3>(3 * (num_contacts_ + i), 3) = matrix3_t::Identity(); 
    C_(2 * dim_contacts_ + i, 6 + 3 * i + 2) = 1.0;                     
  }
  C_.block(0, 6, dim_contacts_, dim_contacts_) = -matrix_t::Identity(dim_contacts_, dim_contacts_);
  
  Q_.setIdentity(num_state_, num_state_);
  P_ = p_scale_init_ * Q_;
  R_.setIdentity(num_observe_, num_observe_);
  feet_heights_.setZero(num_contacts_);
}

vector_t LinearKalmanFilter::update(const rclcpp::Time& time, const rclcpp::Duration& period) {

  discretizeModel(period.seconds(), A_, B_, Q_);
  updateInput(u_);
  updateObserve(z_);
  updateFilter(x_hat_, P_);
  updateLinear(x_hat_.head<3>(), x_hat_.segment<3>(3));

  vector3_t zyx = quatToZyx(quat_) - zyx_offset_;
  vector3_t angular_vel_global = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, 
      getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_));
  updateAngular(zyx, angular_vel_global);
  
  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  return rbd_state_;
}

void LinearKalmanFilter::discretizeModel(const scalar_t dt, matrix_t& A, matrix_t& B, matrix_t& Q) {
  A.block<3, 3>(0, 3) = dt * matrix3_t::Identity();
  
  B.block<3, 3>(0, 0) = 0.5 * dt * dt * matrix3_t::Identity();
  B.block<3, 3>(3, 0) = dt * matrix3_t::Identity();

  Q.block<3, 3>(0, 0) = (dt / q_scale_factor_) * matrix3_t::Identity();
  Q.block<3, 3>(3, 3) = (dt * std::fabs(g_(2)) / q_scale_factor_) * matrix3_t::Identity();
  Q.block(6, 6, dim_contacts_, dim_contacts_) = dt * matrix_t::Identity(dim_contacts_, dim_contacts_);
}

void LinearKalmanFilter::updateInput(vector_t& u) {
  u = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linear_acc_ + g_;
}

void LinearKalmanFilter::updateObserve(vector_t& z) {
  auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();

  vector_t q_pino = vector_t::Zero(info_.generalizedCoordinatesNum);
  vector_t v_pino = vector_t::Zero(info_.generalizedCoordinatesNum);

  q_pino.segment<3>(3) = rbd_state_.head<3>(); // Only set orientation, let position in origin.
  q_pino.tail(info_.actuatedDofNum) = rbd_state_.segment(6, info_.actuatedDofNum);
  v_pino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
                          q_pino.segment<3>(3),
                          rbd_state_.segment<3>(info_.generalizedCoordinatesNum)); // Only set angular velocity, let linear velocity be zero
  v_pino.tail(info_.actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);

  pinocchio::forwardKinematics(model, data, q_pino, v_pino);
  pinocchio::updateFramePlacements(model, data);
  const auto ee_pos = ee_kinematics_->getPosition(vector_t());
  const auto ee_vel = ee_kinematics_->getVelocity(vector_t(), vector_t());

  for (size_t i = 0; i < num_contacts_; ++i) {
    z.segment<3>(3 * i) = -ee_pos[i]; // p - pf
    z.segment<3>(3 * i)[2] += foot_radius_;
    z.segment<3>(12 + 3 * i) = -ee_vel[i];
    z(24 + i) = feet_heights_[i];
  }
}

void LinearKalmanFilter::updateFilter(vector_t& x_hat, matrix_t& P) {
  auto Q = Q_;
  auto R = R_;

  Q.block<3, 3>(0, 0) *= q_base_pos_;
  Q.block<3, 3>(3, 3) *= q_base_vel_;
  Q.block(6, 6, dim_contacts_, dim_contacts_) *= q_ee_pos_static_;

  R.block(0, 0, dim_contacts_, dim_contacts_) *= r_ee_pos_;
  R.block(dim_contacts_, dim_contacts_, dim_contacts_, dim_contacts_) *= r_ee_vel_;
  R.block(2 * dim_contacts_, 2 * dim_contacts_, num_contacts_, num_contacts_) *= r_ee_height_;

  for (size_t i = 0; i < num_contacts_; ++i) {
    scalar_t suspect_factor = contact_flag_[i] ? suspect_factor_contact_ : suspect_factor_no_contact_;

    size_t start_idx_qe = 6 + 3 * i;
    size_t start_idx_rp = 3 * i;
    size_t start_idx_rv = dim_contacts_ + 3 * i;
    size_t start_idx_rz = 2 * dim_contacts_ + i;

    Q.block<3, 3>(start_idx_qe, start_idx_qe) *= suspect_factor;
    R.block<3, 3>(start_idx_rp, start_idx_rp) *= suspect_factor;
    R.block<3, 3>(start_idx_rv, start_idx_rv) *= suspect_factor;
    R(start_idx_rz, start_idx_rz) *= suspect_factor;
  }

  x_hat = A_ * x_hat + B_ * u_;
  P = A_ * P * A_.transpose() + Q;

  auto S = C_ * P * C_.transpose() + R;
  auto K = P * C_.transpose() * S.lu().solve(matrix_t::Identity(num_observe_, num_observe_));

  x_hat += K * (z_ - C_ * x_hat);
  P = (matrix_t::Identity(num_state_, num_state_) - K * C_) * P;
  P = 0.5 * (P + P.transpose());
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
