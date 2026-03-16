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
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>

#include "quad_control_mpc/LeggedRobotInterface.h"
#include "quad_control_ros/gait/GaitReceiver.h"
#include "quad_control_ros/visualization/LeggedRobotVisualizer.h"
#include "quad_control_se/LinearKalmanFilter.hpp"


namespace quad_control {
using namespace ocs2;
using namespace quad_robot;

struct JointHandle {
  std::string name;
  // Read
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> effort;
  // Write
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> pos_des;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> vel_des;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> ff;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> kp;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> kd;
};

struct ImuHandle {
  std::string name;
  // Read
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 4> ori;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 9> ori_cov;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 3> angular_vel;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 9> angular_vel_cov;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 3> linear_acc;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 9> linear_acc_cov;
};

struct ForceTorqueHandle {
  std::string name;
  // Read
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> contact;

  bool incontact() const {
    return contact.get().get_optional<double>().value_or(0.0) > 0.99;
  }
};

class QuadController : public controller_interface::ControllerInterface {
 public:
  QuadController() = default;
  ~QuadController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

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
  void printStateCommand();
  void declareSensorParams(rclcpp_lifecycle::LifecycleNode& node);
  bool loadSensorParams(rclcpp_lifecycle::LifecycleNode& node);
  bool setupJointHandles();
  bool setupIMUHandles();
  bool setupFTHandles();
  void printHandlesCfg();
  
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
  virtual void setupQuadrupedInterface(const std::string& task_file, const std::string& urdf_file, const std::string& reference_file,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimation(const std::string& task_file, bool verbose);
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  std::shared_ptr<LeggedRobotInterface> quadruped_interface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> ee_kinematics_ptr_;

  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;

  SystemObservation current_observation_;
  vector_t measured_rbd_state_;
  std::shared_ptr<StateEstimateBase> state_estimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;

  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;
  std::shared_ptr<LeggedRobotVisualizer> robot_visualizer_;
  // std::shared_ptr<LeggedSelfCollisionVisualization> self_collision_visualization_;

 private:
  const std::string robot_name_ = "quad_robot";
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;

  std::thread mpc_thread_;
  std::atomic_bool controller_running_{}, mpc_running_{};
  benchmark::RepeatedTimer mpc_timer_;
};

}  // namespace quad_control
