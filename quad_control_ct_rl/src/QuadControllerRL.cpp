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

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "quad_control_ct_rl/QuadControllerRL.hpp"


namespace quad_control {


QuadControllerRL::~QuadControllerRL() = default;


controller_interface::CallbackReturn QuadControllerRL::on_init() {
  try {
    node_lifecycle_ = get_node();
    if (!node_lifecycle_) {
      RCLCPP_ERROR(rclcpp::get_logger("QuadControllerRL"), "Failed to get lifecycle node pointer.");
      return controller_interface::CallbackReturn::ERROR;
    }

    declareSensorParams();
    declareFileParams();
    declarePolicyParams();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL on_init succeed.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadControllerRL::on_configure(const rclcpp_lifecycle::State&) {
  if (on_configure_succeed_)
    return controller_interface::CallbackReturn::SUCCESS;

  if (!loadSensorParams()) return controller_interface::CallbackReturn::ERROR;
  if (!loadFileParams()) return controller_interface::CallbackReturn::ERROR;
  if (!loadPolicyParams()) return controller_interface::CallbackReturn::ERROR;

  try {
    if (!setupPolicy()) return controller_interface::CallbackReturn::ERROR;
    setupPolicyIO();

    setupQuadInterface(task_file_, urdf_file_, reference_file_);
    setupStateEstimation();
    setupRbd();

    setupSub();
    setupPub();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to setup QuadControllerRL: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  on_configure_succeed_ = true;
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL on_configure succeed.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadControllerRL::on_activate(const rclcpp_lifecycle::State&) {
  if (!setupJointHandles() || !setupIMUHandles() || !setupFTHandles()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  delay_expired_ = false;
  start_time_ = node_lifecycle_->get_clock()->now();
  printHandlesCfg();
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL on_activate succeed.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadControllerRL::on_deactivate(const rclcpp_lifecycle::State&) {

  joint_handles_.clear();
  ft_handles_.clear();
  imu_handles_.clear();

  // TODO: policy/actions/observations clear()

  RCLCPP_INFO(node_lifecycle_->get_logger(), "Controller deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration QuadControllerRL::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  stateIfConfig(config);
  return config;
}


controller_interface::InterfaceConfiguration QuadControllerRL::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  commandIfConfig(config);
  return config;
}


controller_interface::return_type QuadControllerRL::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (!delay_expired_) {
    auto current_time = node_lifecycle_->get_clock()->now();
    if ((current_time - start_time_).seconds() > delay_duration_)
      delay_expired_ = true;
  }

  updateStateEstimation(time, period);
  computeObservations(); // TODO

  computeActions(); // TODO
  vector_t ff = rbd_torque.tail(12); // TODO
  vector_t pos_des = centroidal_model::getJointAngles(optimized_state, quad_interface_->getCentroidalModelInfo()); // TODO
  vector_t vel_des = centroidal_model::getJointVelocities(optimized_input, quad_interface_->getCentroidalModelInfo()); // TODO
  if (delay_expired_) {
    setCommand(ff, pos_des, vel_des, kp_, kd_);
  }

  auto observation_msg = ros_msg_conversions::createObservationMsg(current_observation_);
  observation_msg.time = time.seconds();
  observation_publisher_->publish(observation_msg);

  printStateCommand(print_period_ms_);
  return controller_interface::return_type::OK;
}


void QuadControllerRL::stateIfConfig(controller_interface::InterfaceConfiguration& config) const {
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


void QuadControllerRL::commandIfConfig(controller_interface::InterfaceConfiguration& config) const {
  for (const auto& joint : joint_names_) {
    for (auto& interface : joint_cmd_interfaces_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
}


void QuadControllerRL::declareSensorParams() {
  node_lifecycle_->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
  node_lifecycle_->declare_parameter<std::vector<std::string>>("imus", std::vector<std::string>());
  node_lifecycle_->declare_parameter<std::vector<std::string>>("feet", std::vector<std::string>());
}


void QuadControllerRL::declareFileParams() {
  node_lifecycle_->declare_parameter<std::string>("task_file", "");
  node_lifecycle_->declare_parameter<std::string>("urdf_file", "");
  node_lifecycle_->declare_parameter<std::string>("reference_file", "");
  node_lifecycle_->declare_parameter<std::string>("policy_file", "");
}


void QuadControllerRL::declarePolicyParams() {
  std::string prefix = "QuadRobotCfg.init_state.default_joint_angle.";
  node_lifecycle_->declare_parameter<double>(prefix + "LF_HAA_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "LF_HFE_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "LF_KFE_joint");
  //
  node_lifecycle_->declare_parameter<double>(prefix + "LH_HAA_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "LH_HFE_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "LH_KFE_joint");
  //
  node_lifecycle_->declare_parameter<double>(prefix + "RF_HAA_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "RF_HFE_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "RF_KFE_joint");
  //
  node_lifecycle_->declare_parameter<double>(prefix + "RH_HAA_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "RH_HFE_joint");
  node_lifecycle_->declare_parameter<double>(prefix + "RH_KFE_joint");

  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.control.stiffness");
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.control.damping");
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.control.action_scale");
  node_lifecycle_->declare_parameter<int>("QuadRobotCfg.control.decimation");
  
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.normalization.obs_scales.lin_vel");
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.normalization.obs_scales.ang_vel");
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.normalization.obs_scales.dof_pos");
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.normalization.obs_scales.dof_vel");

  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.normalization.clip_scales.clip_actions");
  node_lifecycle_->declare_parameter<double>("QuadRobotCfg.normalization.clip_scales.clip_observations");

  node_lifecycle_->declare_parameter<int>("QuadRobotCfg.size.actions_size");
  node_lifecycle_->declare_parameter<int>("QuadRobotCfg.size.observations_size");
}


bool QuadControllerRL::loadSensorParams() {
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


bool QuadControllerRL::loadFileParams() {
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
  if (!node_lifecycle_->get_parameter("policy_file", policy_file_) || policy_file_.empty()) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to load 'policy_file' parameter or the path is empty.");
    return false;
  }

  RCLCPP_INFO(node_lifecycle_->get_logger(), "Loaded files:\nTask: %s\nURDF: %s\nRef: %s", 
              task_file_.c_str(), urdf_file_.c_str(), reference_file_.c_str());
  return true;
}


bool QuadControllerRL::loadPolicyParams() {
  try {
    std::string prefix = "QuadRobotCfg.init_state.default_joint_angle.";
    rl_robot_cfg_.init_state.LF_HAA_joint = node_lifecycle_->get_parameter(prefix + "LF_HAA_joint").as_double();
    rl_robot_cfg_.init_state.LF_HFE_joint = node_lifecycle_->get_parameter(prefix + "LF_HFE_joint").as_double();
    rl_robot_cfg_.init_state.LF_KFE_joint = node_lifecycle_->get_parameter(prefix + "LF_KFE_joint").as_double();
    //
    rl_robot_cfg_.init_state.LH_HAA_joint = node_lifecycle_->get_parameter(prefix + "LH_HAA_joint").as_double();
    rl_robot_cfg_.init_state.LH_HFE_joint = node_lifecycle_->get_parameter(prefix + "LH_HFE_joint").as_double();
    rl_robot_cfg_.init_state.LH_KFE_joint = node_lifecycle_->get_parameter(prefix + "LH_KFE_joint").as_double();
    //
    rl_robot_cfg_.init_state.RF_HAA_joint = node_lifecycle_->get_parameter(prefix + "RF_HAA_joint").as_double();
    rl_robot_cfg_.init_state.RF_HFE_joint = node_lifecycle_->get_parameter(prefix + "RF_HFE_joint").as_double();
    rl_robot_cfg_.init_state.RF_KFE_joint = node_lifecycle_->get_parameter(prefix + "RF_KFE_joint").as_double();
    //
    rl_robot_cfg_.init_state.RH_HAA_joint = node_lifecycle_->get_parameter(prefix + "RH_HAA_joint").as_double();
    rl_robot_cfg_.init_state.RH_HFE_joint = node_lifecycle_->get_parameter(prefix + "RH_HFE_joint").as_double();
    rl_robot_cfg_.init_state.RH_KFE_joint = node_lifecycle_->get_parameter(prefix + "RH_KFE_joint").as_double();

    rl_robot_cfg_.control_cfg.stiffness   = node_lifecycle_->get_parameter("QuadRobotCfg.control.stiffness").as_double();
    rl_robot_cfg_.control_cfg.damping     = node_lifecycle_->get_parameter("QuadRobotCfg.control.damping").as_double();
    rl_robot_cfg_.control_cfg.actionScale = node_lifecycle_->get_parameter("QuadRobotCfg.control.action_scale").as_double();
    rl_robot_cfg_.control_cfg.decimation  = node_lifecycle_->get_parameter("QuadRobotCfg.control.decimation").as_int();

    rl_robot_cfg_.obs_scales.lin_vel = node_lifecycle_->get_parameter("QuadRobotCfg.normalization.obs_scales.lin_vel").as_double();
    rl_robot_cfg_.obs_scales.ang_vel = node_lifecycle_->get_parameter("QuadRobotCfg.normalization.obs_scales.ang_vel").as_double();
    rl_robot_cfg_.obs_scales.dof_pos = node_lifecycle_->get_parameter("QuadRobotCfg.normalization.obs_scales.dof_pos").as_double();
    rl_robot_cfg_.obs_scales.dof_vel = node_lifecycle_->get_parameter("QuadRobotCfg.normalization.obs_scales.dof_vel").as_double();

    rl_robot_cfg_.clip_actions      = node_lifecycle_->get_parameter("QuadRobotCfg.normalization.clip_scales.clip_actions").as_double();
    rl_robot_cfg_.clip_observations = node_lifecycle_->get_parameter("QuadRobotCfg.normalization.clip_scales.clip_observations").as_double();

    actions_size_      = node_lifecycle_->get_parameter("QuadRobotCfg.size.actions_size").as_int();
    observations_size_ = node_lifecycle_->get_parameter("QuadRobotCfg.size.observations_size").as_int();

  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Parameter not declared: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Error loading parameters: %s", e.what());
    return false;
  }
}


bool QuadControllerRL::setupPolicy() {
  onnx_env_ptr_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "QuadControllerRLOnnx");
  Ort::SessionOptions session_options;
  session_options.SetInterOpNumThreads(1);
  session_options.SetIntraOpNumThreads(1);
  try {
    session_ptr_ = std::make_unique<Ort::Session>(*onnx_env_ptr_, policy_file_.c_str(), session_options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), "Failed to create ONNX session: %s", e.what());
    return false;
  }

  input_names_.clear();
  output_names_.clear();
  input_shapes_.clear();
  output_shapes_.clear();

  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t i = 0; i < session_ptr_->GetInputCount(); i++) {
    auto name_ptr = session_ptr_->GetInputNameAllocated(i, allocator);
    input_names_.push_back(std::string(name_ptr.get()));
    input_shapes_.push_back(session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (size_t i = 0; i < session_ptr_->GetOutputCount(); i++) {
    auto name_ptr = session_ptr_->GetOutputNameAllocated(i, allocator);
    output_names_.push_back(std::string(name_ptr.get()));
    output_shapes_.push_back(session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }

  RCLCPP_INFO(node_lifecycle_->get_logger(), "ONNX model loaded successfully from: %s", policy_file_.c_str());

  int64_t model_obs_size = input_shapes_[0].back(); 
  if (model_obs_size != rl_robot_cfg_.observations_size) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), 
                 "Observation size mismatch! YAML: %d, ONNX Model: %ld", 
                 rl_robot_cfg_.observations_size, model_obs_size);
    return false;
  }
  int64_t model_act_size = output_shapes_[0].back();
  if (model_act_size != rl_robot_cfg_.actions_size) {
    RCLCPP_ERROR(node_lifecycle_->get_logger(), 
                 "Action size mismatch! YAML: %d, ONNX Model: %ld", 
                 rl_robot_cfg_.actions_size, model_act_size);
    return false;
  }
  RCLCPP_INFO(node_lifecycle_->get_logger(), 
              "ONNX Model validated: Obs=%ld, Actions=%ld", 
              model_obs_size, model_act_size);

  return true;
}


void QuadControllerRL::setupPolicyIO() {
  actions_.resize(actions_size_);
  observations_.resize(observations_size_);

  std::vector<scalar_t> temp{
      rl_robot_cfg_.init_state.LF_HAA_joint, rl_robot_cfg_.init_state.LF_HFE_joint, rl_robot_cfg_.init_state.LF_KFE_joint,
      rl_robot_cfg_.init_state.LH_HAA_joint, rl_robot_cfg_.init_state.LH_HFE_joint, rl_robot_cfg_.init_state.LH_KFE_joint,
      rl_robot_cfg_.init_state.RF_HAA_joint, rl_robot_cfg_.init_state.RF_HFE_joint, rl_robot_cfg_.init_state.RF_KFE_joint,
      rl_robot_cfg_.init_state.RH_HAA_joint, rl_robot_cfg_.init_state.RH_HFE_joint, rl_robot_cfg_.init_state.RH_KFE_joint};
  const auto& info = quad_interface_->getCentroidalModelInfo();
  default_joint_angles_.resize(info.actuatedDofNum);
  for (size_t i = 0; i < info.actuatedDofNum; i++) {
    default_joint_angles_(i) = temp[i];
  }  

  cmd_vel_.setZero();
  last_actions_.resize(quad_interface_->getCentroidalModelInfo().actuatedDofNum);
}


void QuadControllerRL::setupSub() {
  cmd_vel_subscriber_ = node_lifecycle_->create_subscription<geometry_msgs::Twist>(
    "quad_robot_cmd_vel", 10,
    [this](const geometry_msgs::Twist::SharedPtr msg) {
      cmd_vel_(0) = msg->linear.x;
      cmd_vel_(1) = msg->linear.y;
      cmd_vel_(2) = msg->angular.z;
    });

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL setupSub succeed.");
}


void QuadControllerRL::setupPub() {
  observation_publisher_ = node_lifecycle_->create_publisher<ocs2_msgs::msg::MpcObservation>(
      robot_name_ + "_mpc_observation", 
      rclcpp::SystemDefaultsQoS());
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL setupPub succeed.");
}


bool QuadControllerRL::setupJointHandles() {
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


bool QuadControllerRL::setupIMUHandles() {

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


bool QuadControllerRL::setupFTHandles() {
  if (foot_names_.empty()) return false;
  ft_handles_.clear();

  for (const auto& name : foot_names_) {
    auto contact = find_interface(state_interfaces_, name, foot_state_interfaces_[0]);
    if (!contact) return false;
    ft_handles_.emplace_back(ForceTorqueHandle{name, *contact});
  }

  return true;
}


void QuadControllerRL::printHandlesCfg() {
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


void QuadControllerRL::printStateCommand(int period_ms) {
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
  ss << "  NAME            | POS    | VEL    | EFF    | P_DES  | V_DES  | FF    | KP    | KD\n";
  ss << "  ----------------|--------|--------|--------|--------|--------|-------|-------|-------\n";
  for (const auto& jh : joint_handles_) {
    char buf[256];
    snprintf(buf, sizeof(buf), "  %-15s | %6.2f | %6.2f | %6.2f | %6.2f | %6.2f | %5.1f | %5.1f | %5.1f\n",
             jh.name.c_str(), get_v(jh.position), get_v(jh.velocity), get_v(jh.effort),
             get_v(jh.pos_des), get_v(jh.vel_des), get_v(jh.ff), get_v(jh.kp), get_v(jh.kd));
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
  RCLCPP_INFO_THROTTLE(logger, clock, period_ms, "%s", ss.str().c_str());
}


void QuadControllerRL::setupQuadInterface(const std::string& task_file, 
                                        const std::string& urdf_file, 
                                        const std::string& reference_file) {
  quad_interface_ = std::make_shared<LeggedRobotInterface>(task_file, urdf_file, reference_file);
  
  pinocchio_mapping_ptr_ = std::make_shared<CentroidalModelPinocchioMapping>(
      quad_interface_->getCentroidalModelInfo());

  ee_kinematics_ptr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      quad_interface_->getPinocchioInterface(), 
      *pinocchio_mapping_ptr_,
      quad_interface_->modelSettings().contactNames3DoF);
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL setupQuadInterface succeed.");
}


void QuadControllerRL::setupStateEstimation() {
  state_estimate_ = std::make_shared<LinearKalmanFilter>(node_lifecycle_,
                                                         quad_interface_->getPinocchioInterface(),
                                                         quad_interface_->getCentroidalModelInfo(), 
                                                         *ee_kinematics_ptr_);
  current_observation_.time = 0.0;

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL setupStateEstimation succeed.");
}


void QuadControllerRL::setupRbd() {
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(
      quad_interface_->getPinocchioInterface(),
      quad_interface_->getCentroidalModelInfo());
  
  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadControllerRL setupRbd succeed.");
}


void QuadControllerRL::updateStateEstimation(const rclcpp::Time& time, 
                                           const rclcpp::Duration& period) {
  vector_t joint_pos(joint_handles_.size()), joint_vel(joint_handles_.size());
  contact_flag_t contact_flag;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angular_vel, linear_acc;
  matrix3_t ori_cov, angular_vel_cov, linear_acc_cov;

  getState(joint_pos, joint_vel, 
           contact_flag, 
           quat, 
           angular_vel, linear_acc,
           ori_cov, 
           angular_vel_cov, linear_acc_cov);

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


void QuadControllerRL::getState(
    vector_t& joint_pos, vector_t& joint_vel, 
    contact_flag_t& contact_flag, 
    Eigen::Quaternion<scalar_t>& quat, 
    vector3_t& angular_vel, vector3_t& linear_acc,
    matrix3_t& ori_cov, 
    matrix3_t& angular_vel_cov, matrix3_t& linear_acc_cov) {
  //
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
}


void QuadControllerRL::setCommand(const vector_t& ff, const vector_t& pos_des, const vector_t& vel_des,
                                double kp, double kd) {
  for (size_t i = 0; i < quad_interface_->getCentroidalModelInfo().actuatedDofNum; ++i) {
    (void)joint_handles_[i].pos_des.get().set_value(pos_des(i));
    (void)joint_handles_[i].vel_des.get().set_value(vel_des(i));
    (void)joint_handles_[i].ff.get().set_value(ff(i));
    (void)joint_handles_[i].kp.get().set_value(kp);
    (void)joint_handles_[i].kd.get().set_value(kd);
  } 
}


void QuadCheaterControllerRL::setupStateEstimation() {
  state_estimate_ = std::make_shared<FromTopicStateEstimate>(node_lifecycle_,
                                                            quad_interface_->getPinocchioInterface(),
                                                            quad_interface_->getCentroidalModelInfo(), 
                                                            *ee_kinematics_ptr_);
  current_observation_.time = 0.0;

  RCLCPP_INFO(node_lifecycle_->get_logger(), "QuadCheaterControllerRL setupStateEstimation succeed.");
}


} // namespace quad_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(quad_control::QuadControllerRL, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(quad_control::QuadCheaterControllerRL, controller_interface::ControllerInterface)
