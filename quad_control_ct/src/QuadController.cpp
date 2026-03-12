#include "quad_control_ct/QuadController.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace quad_robot {

QuadController::~QuadController() = default;

controller_interface::InterfaceConfiguration QuadController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  stateIfConfig(config);
  return config;
}

void QuadController::stateIfConfig(controller_interface::InterfaceConfiguration& config) const {
  for (const auto& joint : joint_names_) {
    for (auto& interface : joint_state_interfaces_) {
      config.names.push_back(joint + "/" + interface);
    }
  }

  for (const auto& imu : imu_names_) {
    for (auto& interface : imu_state_interfaces_) {
      config.names.push_back(imu + "/" + interface);
    }
    for (const auto& interface : imu_state_interfaces_cov_) {
      for (size_t idx = 0; idx < 9; ++idx) {
        config.names.push_back(imu + "/" + interface + "." + std::to_string(idx));
      }
    }
  }

  for (const auto& foot : foot_names_) {
    for (const auto& interface : foot_state_interfaces_) {
      config.names.push_back(foot + "/" + interface);
    }
  }
}

controller_interface::InterfaceConfiguration QuadController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  commandIfConfig(config);
  return config;
}

void QuadController::commandIfConfig(controller_interface::InterfaceConfiguration& config) const {
  for (const auto& joint : joint_names_) {
    for (auto& interface : joint_cmd_interfaces_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
}

controller_interface::return_type QuadController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  
  printStateCommand();
  return controller_interface::return_type::OK;
}

void QuadController::printStateCommand() {
  auto logger = get_node()->get_logger();
  auto& clock = *(get_node()->get_clock());
  
  // Lambda
  auto get_v = [](const auto& iface) -> double {
    return iface.get().template get_optional<double>().value_or(-1.0);
  };

  std::stringstream ss;
  ss << "\n==================== QUAD ROBOT STATE MONITOR ====================\n";

  // Joints data
  ss << "[JOINT STATES & COMMANDS]\n";
  ss << "  NAME            | POS    | VEL    | EFF    | P_DES  | KP    | KD\n";
  ss << "  ----------------|--------|--------|--------|--------|-------|-------\n";
  for (const auto& jh : joint_handles_) {
    char buf[256];
    snprintf(buf, sizeof(buf), "  %-15s | %6.2f | %6.2f | %6.2f | %6.2f | %5.1f | %5.1f\n",
             jh.name.c_str(), get_v(jh.position), get_v(jh.velocity), get_v(jh.effort),
             get_v(jh.pos_des), get_v(jh.kp), get_v(jh.kd));
    ss << buf;
  }

  // IMU data
  for (const auto& ih : imu_handles_) {
    ss << "\n[IMU STATE: " << ih.name << "]\n";
    
    char base_buf[256];
    snprintf(base_buf, sizeof(base_buf), 
             "  Orientation (x,y,z,w): [%.3f, %.3f, %.3f, %.3f]\n"
             "  Angular Vel (x,y,z)  : [%.3f, %.3f, %.3f]\n"
             "  Linear Acc  (x,y,z)  : [%.3f, %.3f, %.3f]\n",
             get_v(ih.ori[0]), get_v(ih.ori[1]), get_v(ih.ori[2]), get_v(ih.ori[3]),
             get_v(ih.angular_vel[0]), get_v(ih.angular_vel[1]), get_v(ih.angular_vel[2]),
             get_v(ih.linear_acc[0]), get_v(ih.linear_acc[1]), get_v(ih.linear_acc[2]));
    ss << base_buf;

    auto append_cov = [&](const std::string& label, const auto& cov_arr) {
      ss << "  " << label << ": [";
      for (size_t i = 0; i < 9; ++i) {
        ss << (i > 0 ? ", " : "") << std::fixed << std::setprecision(4) << get_v(cov_arr[i]);
      }
      ss << "]\n";
    };
    append_cov("OriCov", ih.ori_cov);
    append_cov("AngCov", ih.angular_vel_cov);
    append_cov("LinCov", ih.linear_acc_cov);
  }

  // FT data
  ss << "\n[FOOT CONTACTS]\n";
  for (const auto& fh : ft_handles_) {
    double val = get_v(fh.contact);
    ss << "  " << fh.name << ": " << (val > 0.99 ? "CONTACT" : "AIR") << " (raw: " << val << ")\n";
  }

  ss << "==================================================================\n";

  // print once every 1000ms
  RCLCPP_INFO_THROTTLE(logger, clock, 1000, "%s", ss.str().c_str());
}

controller_interface::CallbackReturn QuadController::on_init() {
  auto node = get_node();

  declareSensorParams(*node);
  return controller_interface::CallbackReturn::SUCCESS;
}

void QuadController::declareSensorParams(rclcpp_lifecycle::LifecycleNode& node) {
  node.declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
  node.declare_parameter<std::vector<std::string>>("imus", std::vector<std::string>());
  node.declare_parameter<std::vector<std::string>>("feet", std::vector<std::string>());
}

controller_interface::CallbackReturn QuadController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  if (!loadSensorParams(*node)) return controller_interface::CallbackReturn::ERROR;
  return controller_interface::CallbackReturn::SUCCESS;
}

bool QuadController::loadSensorParams(rclcpp_lifecycle::LifecycleNode& node) {
  if (!node.get_parameter("joints", joint_names_) || joint_names_.empty()) {
    RCLCPP_ERROR(node.get_logger(), "Failed to load 'joints' parameter or the list is empty.");
    return false;
  }
  if (!node.get_parameter("imus", imu_names_) || imu_names_.empty()) {
    RCLCPP_ERROR(node.get_logger(), "Failed to load 'imus' parameter or the list is empty.");
    return false;
  }
  if (!node.get_parameter("feet", foot_names_) || foot_names_.empty()) {
    RCLCPP_ERROR(node.get_logger(), "Failed to load 'feet' parameter or the list is empty.");
    return false;
  }
  RCLCPP_INFO(node.get_logger(), 
              "Parameters loaded successfully: %zu joints, %zu IMUs, %zu feet sensors.", 
              joint_names_.size(), imu_names_.size(), foot_names_.size());
  return true;
}

controller_interface::CallbackReturn QuadController::on_activate(const rclcpp_lifecycle::State&) {
  if (!setupJointHandles() || !setupIMUHandles() || !setupFTHandles()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  printHandlesCfg();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool QuadController::setupJointHandles() {
  if (joint_names_.empty()) return false;
  joint_handles_.clear();

  for (const auto& name : joint_names_) {
    auto pos = find_interface(state_interfaces_, name, joint_state_interfaces_[0]);
    auto vel = find_interface(state_interfaces_, name, joint_state_interfaces_[1]);
    auto eff = find_interface(state_interfaces_, name, joint_state_interfaces_[2]);
    auto p_d = find_interface(command_interfaces_, name, joint_cmd_interfaces_[0]);
    auto v_d = find_interface(command_interfaces_, name, joint_cmd_interfaces_[1]);
    auto ff  = find_interface(command_interfaces_, name, joint_cmd_interfaces_[2]);
    auto kp  = find_interface(command_interfaces_, name, joint_cmd_interfaces_[3]);
    auto kd  = find_interface(command_interfaces_, name, joint_cmd_interfaces_[4]);

    if (!pos || !vel || !eff || !p_d || !v_d || !ff || !kp || !kd) return false;
    joint_handles_.emplace_back(JointHandle{name, *pos, *vel, *eff, *p_d, *v_d, *ff, *kp, *kd});
  }

  return true;
}

bool QuadController::setupIMUHandles() {

  if (imu_names_.empty()) return false;
  imu_handles_.clear();

  for (const auto& name : imu_names_) {
    std::vector<const hardware_interface::LoanedStateInterface*> base_ifs;
    for (const auto& interface : imu_state_interfaces_) {
      auto res = find_interface(state_interfaces_, name, interface);
      if (!res) return false;
      base_ifs.push_back(res);
    }

    std::vector<const hardware_interface::LoanedStateInterface*> cov_ifs;
    for (const auto& interface : imu_state_interfaces_cov_) {
      for (size_t idx = 0; idx < 9; ++idx) {
        auto res = find_interface(state_interfaces_, name, interface + "." + std::to_string(idx));
        if (!res) return false;
        cov_ifs.push_back(res);
      }
    }

    ImuHandle imu{
      name,
      {*base_ifs[0], *base_ifs[1], *base_ifs[2], *base_ifs[3]},
      {*cov_ifs[0], *cov_ifs[1], *cov_ifs[2], *cov_ifs[3], *cov_ifs[4], *cov_ifs[5], *cov_ifs[6], *cov_ifs[7], *cov_ifs[8]},
      {*base_ifs[4], *base_ifs[5], *base_ifs[6]},
      {*cov_ifs[9], *cov_ifs[10], *cov_ifs[11], *cov_ifs[12], *cov_ifs[13], *cov_ifs[14], *cov_ifs[15], *cov_ifs[16], *cov_ifs[17]},
      {*base_ifs[7], *base_ifs[8], *base_ifs[9]},
      {*cov_ifs[18], *cov_ifs[19], *cov_ifs[20], *cov_ifs[21], *cov_ifs[22], *cov_ifs[23], *cov_ifs[24], *cov_ifs[25], *cov_ifs[26]}
    };
    imu_handles_.emplace_back(std::move(imu));
  }

  return true;
}

bool QuadController::setupFTHandles() {
  if (foot_names_.empty()) return false;
  ft_handles_.clear();

  for (const auto& name : foot_names_) {
    auto contact = find_interface(state_interfaces_, name, foot_state_interfaces_[0]);
    if (!contact) return false;
    ft_handles_.emplace_back(ForceTorqueHandle{name, *contact});
  }

  return true;
}

void QuadController::printHandlesCfg() {
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "--------------------------------------------------");
  RCLCPP_INFO(logger, "Quad Controller Activated Successfully");

  RCLCPP_INFO(logger, ">> [Joint Handles]");
  for (const auto& jh : joint_handles_) {
    RCLCPP_INFO(logger, "   - %s (state: pos, vel, eff | command: pos_des, vel_des, ff, kp, kd)", 
                jh.name.c_str());
  }

  RCLCPP_INFO(logger, ">> [IMU Handles]");
  for (const auto& ih : imu_handles_) {
    RCLCPP_INFO(logger, ">> IMU Name: %s", ih.name.c_str());

    std::string ori_str;
    for (const auto& iface : ih.ori) ori_str += (iface.get().get_interface_name() + " ");
    RCLCPP_INFO(logger, "   - Orientation: [ %s]", ori_str.c_str());

    std::string ori_cov_str;
    for (const auto& iface : ih.ori_cov) ori_cov_str += (iface.get().get_interface_name() + " ");
    RCLCPP_INFO(logger, "   - Ori Cov: [ %s]", ori_cov_str.c_str());

    std::string ang_vel_str;
    for (const auto& iface : ih.angular_vel) ang_vel_str += (iface.get().get_interface_name() + " ");
    RCLCPP_INFO(logger, "   - Angular Vel: [ %s]", ang_vel_str.c_str());

    std::string ang_vel_cov_str;
    for (const auto& iface : ih.angular_vel_cov) ang_vel_cov_str += (iface.get().get_interface_name() + " ");
    RCLCPP_INFO(logger, "   - AngVel Cov: [ %s]", ang_vel_cov_str.c_str());

    std::string lin_acc_str;
    for (const auto& iface : ih.linear_acc) lin_acc_str += (iface.get().get_interface_name() + " ");
    RCLCPP_INFO(logger, "   - Linear Acc: [ %s]", lin_acc_str.c_str());

    std::string lin_acc_cov_str;
    for (const auto& iface : ih.linear_acc_cov) lin_acc_cov_str += (iface.get().get_interface_name() + " ");
    RCLCPP_INFO(logger, "   - LinAcc Cov: [ %s]", lin_acc_cov_str.c_str());
  }

  RCLCPP_INFO(logger, ">> [FT Handles]");
  for (const auto& ch : ft_handles_) {
    RCLCPP_INFO(logger, "   - %s -> %s/%s", 
                ch.name.c_str(),
                ch.contact.get().get_prefix_name().c_str(), 
                ch.contact.get().get_interface_name().c_str());
  }
  RCLCPP_INFO(logger, "--------------------------------------------------");
}

controller_interface::CallbackReturn QuadController::on_deactivate(const rclcpp_lifecycle::State&) {
  joint_handles_.clear();
  ft_handles_.clear();
  imu_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace quad_robot

PLUGINLIB_EXPORT_CLASS(quad_robot::QuadController, controller_interface::ControllerInterface)
