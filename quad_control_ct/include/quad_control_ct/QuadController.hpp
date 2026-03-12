#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control 核心
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
// 这里的 HybridJointInterface 和 ContactSensorInterface 需要根据你 ROS 2 版本的实现包含对应头文件
#include <legged_common/hardware_interface/hybrid_joint_interface.hpp> 
#include <legged_common/hardware_interface/contact_sensor_interface.hpp>

// OCS2 & Legged 逻辑 (保持逻辑引用)
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// ROS 2 Jazzy 中建议直接继承 ControllerInterface
class LeggedController : public controller_interface::ControllerInterface {
 public:
  LeggedController() = default;
  ~LeggedController() override;

  // ROS 2 生命周期方法
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 protected:
  // 内部逻辑函数，将 ros::Time 替换为 rclcpp::Time
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // 接口 Handles (ROS 2 中存储的是引用或包装类)
  // 注意：Jazzy 中如果是 bool 类型，可以通过 handle.get_value<bool>() 获取
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> hybridJointCommandHandles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> hybridJointStateHandles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> contactHandles_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> imuSensorHandle_;

  // OCS2 核心组件 (保持不变)
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // 线程管理 (ROS 2 建议使用原生 thread 或 Executor)
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
};

}  // namespace legged
