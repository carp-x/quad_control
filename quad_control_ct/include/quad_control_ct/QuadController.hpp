#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"


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

  bool contact() const {
    return contact.get().get_value() > 0.99;
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
  std::unique_ptr<ImuHandle> imu_handle_;
  std::vector<ForceTorqueHandle> contact_handles_;

  std::vector<std::string> joint_names_ = {
      "LF_HAA_LF_HIP", "LF_HFE_LF_THIGH", "LF_KFE_LF_SHANK",
      "RF_HAA_RF_HIP", "RF_HFE_RF_THIGH", "RF_KFE_RF_SHANK",
      "LH_HAA_LH_HIP", "LH_HFE_LH_THIGH", "LH_KFE_LH_SHANK",
      "RH_HAA_RH_HIP", "RH_HFE_RH_THIGH", "RH_KFE_RH_SHANK"};
  std::vector<std::string> foot_names_ = {"LF_ft", "RF_ft", "LH_ft", "RH_ft"};
  std::string imu_name_ = "base_imu";

 private:
  bool setup_joint_handles();
  bool setup_imu_handles();
  bool setup_contact_handles();

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
