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

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <thread>

#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <Eigen/Geometry>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include "quad_control_se/LinearKalmanFilter.hpp"
#include "quad_control_se/FromTopiceEstimate.hpp"

#include "quad_control_ct/HardwareInterfaceHandles.hpp"
#include "quad_control_ct_rl/RLRobotCfg.hpp"

namespace quad_control {
using namespace ocs2;
using namespace quad_robot;

class QuadControllerRL : public controller_interface::ControllerInterface {
 public:
  QuadControllerRL() = default;
  ~QuadControllerRL() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  std::vector<JointHandle> joint_handles_;
  std::vector<ImuHandle> imu_handles_;
  std::vector<ForceTorqueHandle> ft_handles_;

  std::vector<std::string> joint_names_;
  const std::vector<std::string> joint_state_interfaces_{"position", "velocity", "effort"};
  const std::vector<std::string> joint_cmd_interfaces_{"pos_des", "vel_des", "ff", "kp", "kd"};
  
  std::vector<std::string> imu_names_;
  const std::vector<std::string> imu_state_interfaces_{
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",    
  };
  const std::vector<std::string> imu_state_interfaces_cov_{
    "orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance",
  };
  
  std::vector<std::string> foot_names_;
  const std::vector<std::string> foot_state_interfaces_{"contact"};
  
 private:
  void stateIfConfig(controller_interface::InterfaceConfiguration& config) const;
  void commandIfConfig(controller_interface::InterfaceConfiguration& config) const;
  void declareSensorParams();
  void declareFileParams();
  bool loadSensorParams();
  bool loadFileParams();
  bool setupJointHandles();
  bool setupIMUHandles();
  bool setupFTHandles();
  void printHandlesCfg();
  void printStateCommand(int period_ms);
  
  template <typename T>
  T* find_interface(std::vector<T>& interfaces, 
                    const std::string& name, 
                    const std::string& interface_name) {
    auto it = std::find_if(interfaces.begin(), interfaces.end(),
                           [&](const T& iface) {
                             return iface.get_prefix_name() == name && 
                                    iface.get_interface_name() == interface_name;
                           });
    return (it != interfaces.end()) ? &(*it) : nullptr;
  }

 protected:
  virtual void declarePolicyParams();
  virtual bool loadPolicyParams();
  virtual bool setupPolicy();
  virtual void setupPolicyIO();
  virtual void setupQuadInterface(const std::string& task_file,
                                  const std::string& urdf_file,
                                  const std::string& reference_file);
  virtual void setupStateEstimation();
  virtual void setupRbd();
  virtual void setupSub();
  virtual void setupPub();
  virtual void updateStateEstimation(const rclcpp::Time& time, 
                                     const rclcpp::Duration& period);
  virtual void computeObservations();
  virtual void computeActions();

  void getState(vector_t& joint_pos, vector_t& joint_vel, 
                contact_flag_t& contact_flag, 
                Eigen::Quaternion<scalar_t>& quat, 
                vector3_t& angular_vel, vector3_t& linear_acc,
                matrix3_t& ori_cov, 
                matrix3_t& angular_vel_cov, matrix3_t& linear_acc_cov);
  void setCommand(const vector_t& ff, const vector_t& pos_des, const vector_t& vel_des,
                  double kp, double kd);

  bool on_configure_succeed_ = false;

  std::string policy_file_;
  RLRobotCfg rl_robot_cfg_{};
  int actions_size_;
  int observations_size_;
  vector_t default_joint_angles_;
  // onnx
  std::shared_ptr<Ort::Env> onnx_env_ptr_;
  std::unique_ptr<Ort::Session> session_ptr_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;
  std::vector<tensor_element_t> actions_;
  std::vector<tensor_element_t> observations_;

  std::string task_file_, urdf_file_, reference_file_;
  std::shared_ptr<LeggedRobotInterface> quad_interface_;
  std::shared_ptr<CentroidalModelPinocchioMapping>  pinocchio_mapping_ptr_;
  std::shared_ptr<PinocchioEndEffectorKinematics> ee_kinematics_ptr_;
  SystemObservation current_observation_;
  vector_t measured_rbd_state_;
  std::shared_ptr<StateEstimateBase> state_estimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;

  vector3_t cmd_vel_;
  vector_t last_actions_;
  int64_t loop_cnt_;

  rclcpp::Subscription<geometry_msgs::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;

  const std::string robot_name_ = "quad_robot";
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;

  bool delay_expired_;
  rclcpp::Time start_time_;
  const double delay_duration_ = 0.0;
  const int print_period_ms_ = 1000;

 private:
  benchmark::RepeatedTimer policy_timer_;
};

class QuadCheaterControllerRL : public QuadControllerRL {
 protected:
  void setupStateEstimation() override;
};

}  // namespace quad_control
