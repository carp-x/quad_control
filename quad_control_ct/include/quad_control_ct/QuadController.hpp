#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_publisher.hpp"


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

namespace quad_robot {

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
  bool setupJointHandles();
  bool setupIMUHandles();
  bool setupFTHandles();

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
};

}  // namespace quad_robot
