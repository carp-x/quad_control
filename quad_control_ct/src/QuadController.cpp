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

#include <sched.h>
#include <pthread.h>
#include <chrono>

#include "quad_control_ct/QuadController.hpp"


namespace quad_control {


QuadController::~QuadController() = default;


controller_interface::CallbackReturn QuadController::on_init() {
  try {
    node_lifecycle_ = get_node();
    if (!node_lifecycle_) {
      RCLCPP_ERROR(rclcpp::get_logger("QuadController"), "Failed to get lifecycle node pointer.");
      return controller_interface::CallbackReturn::ERROR;
    }

    declareSensorParams();
    declareFileParams();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController on_init succeed.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadController::on_configure(const rclcpp_lifecycle::State&) {
  node_base_ = std::make_shared<rclcpp::Node>(robot_name_);

  if (!loadSensorParams()) return controller_interface::CallbackReturn::ERROR;
  if (!loadFileParams()) return controller_interface::CallbackReturn::ERROR;

  try {
    setupQuadInterface(task_file_, urdf_file_, reference_file_);
    setupMpc();
    setupMrt();
    setupStateEstimation(task_file_);
    setupRbd();
    setupSub();
    setupPub();
    setupVisualization();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to setup QuadController: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController on_configure succeed.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadController::on_activate(const rclcpp_lifecycle::State&) {
  if (!setupJointHandles() || !setupIMUHandles() || !setupFTHandles()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  activateMrt();

  printHandlesCfg();
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController on_activate succeed.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadController::on_deactivate(const rclcpp_lifecycle::State&) {
  mpc_running_ = false;
  controller_running_ = false;

  if (mpc_thread_.joinable()) {
    mpc_thread_.join();
  }

  joint_handles_.clear();
  ft_handles_.clear();
  imu_handles_.clear();

  RCLCPP_INFO(node_lifecycle_->get_logger(), "Controller deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration QuadController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  stateIfConfig(config);
  return config;
}


controller_interface::InterfaceConfiguration QuadController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  commandIfConfig(config);
  return config;
}


controller_interface::return_type QuadController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  
  printStateCommand();
  return controller_interface::return_type::OK;
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


void QuadController::commandIfConfig(controller_interface::InterfaceConfiguration& config) const {
  for (const auto& joint : joint_names_) {
    for (auto& interface : joint_cmd_interfaces_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
}


void QuadController::printStateCommand() {
  auto logger = node_lifecycle_->get_logger();
  auto& clock = *(node_lifecycle_->get_clock());
  
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


void QuadController::declareSensorParams() {
  node_lifecycle_->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
  node_lifecycle_->declare_parameter<std::vector<std::string>>("imus", std::vector<std::string>());
  node_lifecycle_->declare_parameter<std::vector<std::string>>("feet", std::vector<std::string>());
}


void QuadController::declareFileParams() {
  node_lifecycle_->declare_parameter<std::string>("task_file", "");
  node_lifecycle_->declare_parameter<std::string>("urdf_file", "");
  node_lifecycle_->declare_parameter<std::string>("reference_file", "");
}


bool QuadController::loadSensorParams() {
  if (!node_lifecycle_->get_parameter("joints", joint_names_) || joint_names_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'joints' parameter or the list is empty.");
    return false;
  }
  if (!node_lifecycle_->get_parameter("imus", imu_names_) || imu_names_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'imus' parameter or the list is empty.");
    return false;
  }
  if (!node_lifecycle_->get_parameter("feet", foot_names_) || foot_names_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'feet' parameter or the list is empty.");
    return false;
  }
  RCLCPP_INFO(node_lifecycle_->get_logger(), 
              "Parameters loaded successfully: %zu joints, %zu IMUs, %zu feet sensors.", 
              joint_names_.size(), imu_names_.size(), foot_names_.size());
  return true;
}


bool QuadController::loadFileParams() {
  if (!node_lifecycle_->get_parameter("task_file", task_file_) || task_file_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'task_file' parameter or the path is empty.");
    return false;
  }
  if (!node_lifecycle_->get_parameter("urdf_file", urdf_file_) || urdf_file_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'urdf_file' parameter or the path is empty.");
    return false;
  }
  if (!node_lifecycle_->get_parameter("reference_file", reference_file_) || reference_file_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'reference_file' parameter or the path is empty.");
    return false;
  }

  RCLCPP_INFO(node_lifecycle_->get_logger(), "Loaded files:\nTask: %s\nURDF: %s\nRef: %s", 
              task_file_.c_str(), urdf_file_.c_str(), reference_file_.c_str());
  return true;
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
  auto logger = node_lifecycle_->get_logger();
  RCLCPP_INFO(logger, "--------------------------------------------------");

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


void QuadController::setupQuadInterface(const std::string& task_file, 
                                        const std::string& urdf_file, 
                                        const std::string& reference_file) {
  quad_interface_ = std::make_shared<LeggedRobotInterface>(task_file, urdf_file, reference_file);
  
  pinocchio_mapping_ptr_ = std::make_shared<CentroidalModelPinocchioMapping>(
      quad_interface_->getCentroidalModelInfo());

  ee_kinematics_ptr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      quad_interface_->getPinocchioInterface(), 
      *pinocchio_mapping_ptr_,
      quad_interface_->modelSettings().contactNames3DoF);
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupQuadInterface succeed.");
}


void QuadController::setupMpc() {
  auto ros_reference_manager_ptr = std::make_shared<RosReferenceManager>(
      robot_name_, quad_interface_->getReferenceManagerPtr());
  ros_reference_manager_ptr->subscribe(node_base_);

  auto gait_receiver_ptr = std::make_shared<GaitReceiver>(
      node_base_, quad_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_name_);
  
  mpc_ = std::make_shared<SqpMpc>(quad_interface_->mpcSettings(), 
                                  quad_interface_->sqpSettings(),
                                  quad_interface_->getOptimalControlProblem(), 
                                  quad_interface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_ptr);
  mpc_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr);

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupMpc succeed.");
}


void QuadController::setupMrt() {
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&quad_interface_->getRollout());
  mpc_timer_.reset();

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupMrt succeed.");
}


void QuadController::activateMrt() {
  if (mpc_thread_.joinable()) {
      controller_running_ = false;
      mpc_thread_.join();
  }
  controller_running_ = true;
  mpc_running_ = false;
  mpc_thread_ = std::thread([this]() {
    double desired_frequency = quad_interface_->mpcSettings().mpcDesiredFrequency_;
    auto sleep_duration = std::chrono::microseconds(static_cast<int>(1e6 / desired_frequency));

    while (rclcpp::ok() && controller_running_) {
      auto start = std::chrono::steady_clock::now();
      try {
        if (mpc_running_) {
          mpc_timer_.startTimer();
          mpc_mrt_interface_->advanceMpc();
          mpc_timer_.endTimer();
        }
      } catch (const std::exception& e) {
        controller_running_ = false;
        RCLCPP_ERROR(node_lifecycle_->get_logger(), "[OCS2 MPC thread] Error: %s", e.what());
      }
      auto end = std::chrono::steady_clock::now();
      auto elapsed = end - start;
      if (sleep_duration > elapsed) {
        std::this_thread::sleep_for(sleep_duration - elapsed);
      }
    }
  });

  int priority = quad_interface_->sqpSettings().threadPriority;
  if (priority > 0) {
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(mpc_thread_.native_handle(), SCHED_FIFO, &param) != 0) {
      RCLCPP_WARN(node_lifecycle_->get_logger(), "Failed to set MPC thread priority to %d", priority);
    } else {
      RCLCPP_INFO(node_lifecycle_->get_logger(), "Successfully set MPC thread priority to %d", priority);
    }
  }

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController activateMrt succeed.");
}


void QuadController::setupStateEstimation(const std::string& task_file) {
  state_estimate_ = std::make_shared<LinearKalmanFilter>(node_lifecycle_,
                                                         quad_interface_->getPinocchioInterface(),
                                                         quad_interface_->getCentroidalModelInfo(), 
                                                         *ee_kinematics_ptr_);
  current_observation_.time = 0.0;

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupStateEstimation succeed.");
}


void QuadController::setupRbd() {
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(
      quad_interface_->getPinocchioInterface(),
      quad_interface_->getCentroidalModelInfo());
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupRbd succeed.");
}


void QuadController::setupSub() {

}


void QuadController::setupPub() {
  observation_publisher_ = node_lifecycle_->create_publisher<ocs2_msgs::msg::MpcObservation>(
      robot_name_ + "_mpc_observation", 
      rclcpp::SystemDefaultsQoS());
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupPub succeed.");
}


void QuadController::setupVisualization() {
  robot_visualizer_ = std::make_shared<LeggedRobotVisualizer>(
      quad_interface_->getPinocchioInterface(), 
      quad_interface_->getCentroidalModelInfo(),
      *ee_kinematics_ptr_, node_base_);
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadController setupVisualization succeed.");
}


void QuadController::updateStateEstimation(const rclcpp::Time& time, 
                                           const rclcpp::Duration& period) {
  vector_t joint_pos(joint_handles_.size()), joint_vel(joint_handles_.size());
  contact_flag_t contact_flag;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angular_vel, linear_acc;
  matrix3_t ori_cov, angular_vel_cov, linear_acc_cov;

  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    joint_pos(i) = joint_handles_[i].position.get().get_optional().value();
    joint_vel(i) = joint_handles_[i].velocity.get().get_optional().value();
  }
  for (size_t i = 0; i < ft_handles_.size(); ++i) {
    contact_flag[i] = ft_handles_[i].incontact();
  }
  auto& ih = imu_handles_[0];
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = ih.ori[i].get().get_optional().value();
  }
  for (size_t i = 0; i < 3; ++i) {
    angular_vel(i) = ih.angular_vel[i].get().get_optional().value();
    linear_acc(i) = ih.linear_acc[i].get().get_optional().value();
  }
  for (size_t i = 0; i < 9; ++i) {
    ori_cov(i) = ih.ori_cov[i].get().get_optional().value();
    angular_vel_cov(i) = ih.angular_vel_cov[i].get().get_optional().value();
    linear_acc_cov(i) = ih.linear_acc_cov[i].get().get_optional().value();
  }

  state_estimate_->updateJointStates(joint_pos, joint_vel);
  state_estimate_->updateContact(contact_flag);
  state_estimate_->updateImu(quat, angular_vel, linear_acc, ori_cov, angular_vel_cov, linear_acc_cov);
  measured_rbd_state_ = state_estimate_->update(time, period);

  scalar_t yaw_last = current_observation_.state(9);  
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
  current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, current_observation_.state(9));
  current_observation_.mode = state_estimate_->getMode();
  current_observation_.time += period.seconds();
}


} // namespace quad_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(quad_control::QuadController, controller_interface::ControllerInterface)
