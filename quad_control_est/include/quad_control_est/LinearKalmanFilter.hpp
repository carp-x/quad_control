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

#pragma once

#include "quad_control_est/StateEstimateBase.hpp"

namespace quad_control {

class LinearKalmanFilter : public StateEstimateBase {
 public:
  LinearKalmanFilter(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr,
                     PinocchioInterface pinocchio_interface,
                     CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics& ee_kinematics);
  
  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
 protected:
  nav_msgs::msg::Odometry getOdomMsg();
  
  const scalar_t q_base_pos_      = 0.01;
  const scalar_t q_base_vel_      = 0.1;
  const scalar_t q_ee_pos_static_ = 0.001;

  const scalar_t r_ee_pos_        = 0.01;
  const scalar_t r_ee_vel_        = 0.5;
  const scalar_t r_ee_height_     = 0.005;
  
  const scalar_t foot_radius_ = 0.08;
  vector_t feet_heights_;

  const scalar_t p_scale_init_ = 1.0;
  const scalar_t q_scale_factor_ = 20.0;
  const vector3_t g_{0, 0, -9.81};
  const scalar_t suspect_factor_contact_ = 1.0;
  const scalar_t suspect_factor_no_contact_ = 1000.0;
  
 private:
  void discretizeModel(const scalar_t dt, matrix_t& A, matrix_t& B, matrix_t& Q);
  void updateInput(vector_t& u);
  void updateObserve(vector_t& z);
  void updateFilter(vector_t& x_hat, matrix_t& P);

  size_t num_contacts_, dim_contacts_, num_state_, num_observe_;
  
  matrix_t A_, B_, C_, Q_, P_, R_;
  vector_t x_hat_, u_, z_;
};

} // namespace quad_control
