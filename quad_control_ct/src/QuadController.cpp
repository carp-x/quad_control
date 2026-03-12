#include "legged_controllers/LeggedController.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace legged {

controller_interface::CallbackReturn LeggedController::on_init() {
  auto node = get_node();
  // 1. 加载参数
  std::string urdfFile = node->declare_parameter<std::string>("urdfFile", "");
  std::string taskFile = node->declare_parameter<std::string>("taskFile", "");
  std::string referenceFile = node->declare_parameter<std::string>("referenceFile", "");

  // 2. 初始化 OCS2 接口
  setupLeggedInterface(taskFile, urdfFile, referenceFile, false);
  setupMpc();
  setupMrt();
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> joints = {"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                     "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& j : joints) {
    config.names.push_back(j + "/position");
    config.names.push_back(j + "/velocity");
    config.names.push_back(j + "/kp");
    config.names.push_back(j + "/kd");
    config.names.push_back(j + "/feedforward_torque");
  }
  return config;
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 添加 Joint, IMU 和 Contact 状态接口名
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    config.names.push_back(name + "/contact");
  }
  return config;
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State&) {
  // 建立映射索引，提高 update 效率
  for (size_t i = 0; i < 12; ++i) {
    jointCommandIdx_.push_back({i*5, i*5+1, i*5+2, i*5+3, i*5+4});
  }
  
  // 启动 MPC 线程逻辑
  mpcRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (mpcRunning_ && rclcpp::ok()) {
      mpcMrtInterface_->advanceMpc();
    }
  });
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // 1. 状态估计 (内部通过 state_interfaces_ 获取数据)
  updateStateEstimation(time, period);

  // 2. 运行 MPC & WBC
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->updatePolicy();
  
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
  
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.seconds());
  vector_t torque = x.tail(12);
  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // 3. 写入硬件命令 (Jazzy 接口赋值)
  for (size_t i = 0; i < 12; ++i) {
    command_interfaces_[jointCommandIdx_[i].pos].set_value(posDes(i));
    command_interfaces_[jointCommandIdx_[i].vel].set_value(velDes(i));
    command_interfaces_[jointCommandIdx_[i].kp].set_value(50.0); // 示例值
    command_interfaces_[jointCommandIdx_[i].kd].set_value(1.0);
    command_interfaces_[jointCommandIdx_[i].feedforward].set_value(torque(i));
  }

  return controller_interface::return_type::OK;
}

void LeggedController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
  vector_t jointPos(12), jointVel(12);
  contact_flag_t contactFlag;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  // 1. 获取关节数据 (假设每个关节顺序存了 pos, vel)
  for (size_t i = 0; i < 12; ++i) {
    jointPos(i) = state_interfaces_[jointStateIdx_[i].pos].get_value();
    jointVel(i) = state_interfaces_[jointStateIdx_[i].vel].get_value();
  }

  // 2. 获取接触传感器数据 (Jazzy 即使是 bool 也通过 get_value() 转 double)
  for (size_t i = 0; i < contactFlag.size(); ++i) {
    contactFlag[i] = static_cast<bool>(state_interfaces_[contactStateIdx_[i]].get_value());
  }

  // 3. 获取 IMU 数据 ( orientation, angularVel, linearAccel )
  // 假设 imuIdx_ 结构体存了这些分量的起始索引
  quat.x() = state_interfaces_[imuIdx_.ori_x].get_value();
  quat.y() = state_interfaces_[imuIdx_.ori_y].get_value();
  quat.z() = state_interfaces_[imuIdx_.ori_z].get_value();
  quat.w() = state_interfaces_[imuIdx_.ori_w].get_value();

  for (size_t i = 0; i updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.seconds();
  
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  mpcRunning_ = false;
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  // 使用 RCLCPP 打印统计信息
  auto logger = rclcpp::get_logger("LeggedController");
  RCLCPP_INFO(logger, "### MPC Benchmarking Average: %.2f ms", mpcTimer_.getAverageInMilliseconds());
  RCLCPP_INFO(logger, "### WBC Benchmarking Average: %.2f ms", wbcTimer_.getAverageInMilliseconds());
}
void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  auto node = get_node();

  // ROS 2 GaitReceiver 和 ReferenceManager 需要 Node 引用
  auto gaitReceiverPtr = std::make_shared<GaitReceiver>(node, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  
  rosReferenceManagerPtr->subscribe(node);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  
  // ROS 2 Publisher
  observationPublisher_ = node->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_ && rclcpp::ok()) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mrtDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(get_node()->get_logger(), "[Ocs2 MPC thread] Error: %s", e.what());
      }
    }
  });
  // 设置线程优先级 (Jazzy/Ubuntu Noble 环境)
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}


} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
