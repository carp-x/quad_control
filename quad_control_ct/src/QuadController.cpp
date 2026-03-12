#include "quad_robot/QuadController.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace quad_robot {

QuadController::~QuadController() = default;

controller_interface::CallbackReturn QuadController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration QuadController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // 关节状态
  for (const auto& joint : joint_names_) {
    for (auto& interface : {"position", "velocity", "effort"}) {
      config.names.push_back(joint + "/" + interface);
    }
  }

  // IMU 状态 (对应 xacro 定义)
  std::vector<std::string> imu_interfaces = {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
                                            "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
                                            "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
  for (const auto& interface : imu_interfaces) config.names.push_back(imu_name_ + "/" + interface);

  // 足端接触
  for (const auto& foot : foot_names_) config.names.push_back(foot + "/contact");

  return config;
}

controller_interface::InterfaceConfiguration QuadController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    for (auto& interface : {"pos_des", "vel_des", "ff", "kp", "kd"}) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  return config;
}

controller_interface::return_type QuadController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  // 1. 读取数据示例
  for (const auto& joint : joint_handles_) {
    double q = joint.position.get().get_value();
  }

  for (const auto& foot : contact_handles_) {
    if (foot.contact()) { /* 落地逻辑 */ }
  }

  // 2. 写入指令示例
  for (auto& joint : joint_handles_) {
    joint.pos_des.get().set_value(0.0);
    joint.kp.get().set_value(50.0);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn QuadController::on_configure(const rclcpp_lifecycle::State&) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadController::on_activate(const rclcpp_lifecycle::State&) {
  if (!setup_joint_handles() || !setup_imu_handles() || !setup_contact_handles()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Quad Controller Activated Successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool QuadController::setup_joint_handles() {
  joint_handles_.clear();
  for (const auto& name : joint_names_) {
    auto pos = find_interface(state_interfaces_, name, "position");
    auto vel = find_interface(state_interfaces_, name, "velocity");
    auto eff = find_interface(state_interfaces_, name, "effort");
    auto p_d = find_interface(command_interfaces_, name, "pos_des");
    auto v_d = find_interface(command_interfaces_, name, "vel_des");
    auto ff  = find_interface(command_interfaces_, name, "ff");
    auto kp  = find_interface(command_interfaces_, name, "kp");
    auto kd  = find_interface(command_interfaces_, name, "kd");

    if (!pos || !vel || !eff || !p_d || !v_d || !ff || !kp || !kd) return false;
    joint_handles_.emplace_back(JointHandle{name, *pos, *vel, *eff, *p_d, *v_d, *ff, *kp, *kd});
  }
  return true;
}

bool QuadController::setup_contact_handles() {
  contact_handles_.clear();
  for (const auto& name : foot_names_) {
    auto contact = find_interface(state_interfaces_, name, "contact");
    if (!contact) return false;
    contact_handles_.emplace_back(ForceTorqueHandle{name, *contact});
  }
  return true;
}

bool QuadController::setup_imu_handles() {
  // 简化的 IMU 绑定逻辑 (ori, ang_vel, lin_acc)
  // 此处由于 ImuHandle 结构复杂，通常需要按顺序循环填充 ori[4], angular_vel[3] 等
  // 示例中略去协方差的循环绑定逻辑
  return true; 
}

controller_interface::CallbackReturn QuadController::on_deactivate(const rclcpp_lifecycle::State&) {
  joint_handles_.clear();
  contact_handles_.clear();
  imu_handle_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace quad_robot

PLUGINLIB_EXPORT_CLASS(quad_robot::QuadController, controller_interface::ControllerInterface)
