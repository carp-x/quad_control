// Copyright 2026 Yuxin Li
//
// Copyright 2025 AIT Austrian Institute of Technology GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// NOTICE: This file has been modified by Yuxin Li in 2026.

#include "quad_control_gz/QuadGzSimSystem.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/sim/components/Name.hh>
#include <gz/physics/Geometry.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>

#include <gz/sim/components/JointVelocityCmd.hh>

#include "control_toolbox/low_pass_filter.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

/******************************************************************************************************/
#include <gz/sim/components/JointForceCmd.hh>

#include <gz/transport/Node.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/msgs/wrench.pb.h>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::
/******************************************************************************************************/

namespace quad_robot {

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /******************************************************************************************************/
  /// \brief Target position command (pos_des)
  double joint_pos_des;

  /// \brief Target velocity command (vel_des)
  double joint_vel_des;

  /// \brief Feedforward effort command (ff)
  double joint_ff;

  /// \brief Proportional gain command (kp)
  double joint_kp;
  
  /// \brief Derivative gain command (kd)
  double joint_kd;
  /******************************************************************************************************/
  
  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// damping_frequency for a first-order low pass
  std::unique_ptr<control_toolbox::LowPassFilter<double>> lpf;
};

/******************************************************************************************************/
class ImuData 
{
public:
  std::string name{};
  sim::Entity sim_imu = sim::kNullEntity;
  std::string topic_name{};
  void OnIMU(const GZ_MSGS_NAMESPACE IMU & msg);

  std::array<double, 4> ori;
  std::array<double, 9> ori_cov;
  std::array<double, 3> angular_vel;
  std::array<double, 9> angular_vel_cov;
  std::array<double, 3> linear_acc;
  std::array<double, 9> linear_acc_cov;
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU & msg)
{
  this->ori[0] = msg.orientation().x();
  this->ori[1] = msg.orientation().y();
  this->ori[2] = msg.orientation().z();
  this->ori[3] = msg.orientation().w();
  this->angular_vel[0] = msg.angular_velocity().x();
  this->angular_vel[1] = msg.angular_velocity().y();
  this->angular_vel[2] = msg.angular_velocity().z();
  this->linear_acc[0] = msg.linear_acceleration().x();
  this->linear_acc[1] = msg.linear_acceleration().y();
  this->linear_acc[2] = msg.linear_acceleration().z();
}

class ForceTorqueData
{
public:
  std::string name{};
  sim::Entity sim_ft = sim::kNullEntity;
  std::string topic_name{};
  void OnForceTorque(const GZ_MSGS_NAMESPACE Wrench & msg);

    // std::array<double, 6> f_t;
  double contact = false;
};

void ForceTorqueData::OnForceTorque(const GZ_MSGS_NAMESPACE Wrench & msg)
{
    // this->f_t[0] = msg.force().x();
    // this->f_t[1] = msg.force().y();
    // this->f_t[2] = msg.force().z();
    // this->f_t[3] = msg.torque().x();
    // this->f_t[4] = msg.torque().y();
    // this->f_t[5] = msg.torque().z();

  const double to_contact_th = 10.0;
  const double to_no_contact_th = 4.0;

  double force_z = std::abs(msg.force().z());
  if (force_z > to_contact_th) this->contact = true;
  else if (force_z < to_no_contact_th) this->contact = false;
}
/******************************************************************************************************/

class QuadGzSimSystemPrivate
{
public:
  QuadGzSimSystemPrivate() = default;

  ~QuadGzSimSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /******************************************************************************************************/
  GZ_TRANSPORT_NAMESPACE Node node;
  std::vector<std::shared_ptr<ImuData>> imus_;
  std::vector<std::shared_ptr<ForceTorqueData>> fts_;
  /******************************************************************************************************/

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  unsigned int update_rate;
};

bool QuadGzSimSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int update_rate)
{
  this->dataPtr = std::make_unique<QuadGzSimSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = update_rate;

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);
    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;
    this->dataPtr->joints_[j].joint_type = _ecm.Component<sim::components::JointType>(
      simjoint)->Data();
    this->dataPtr->joints_[j].joint_axis = _ecm.Component<sim::components::JointAxis>(
      simjoint)->Data();

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointTransmittedWrench().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value =
      [this, joint_name](const hardware_interface::InterfaceInfo & interface_info) {
        double initial_value{0.0};
        if (!interface_info.initial_value.empty()) {
          try {
            initial_value = hardware_interface::stod(interface_info.initial_value);
            RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
          } catch (std::invalid_argument &) {
            RCLCPP_ERROR_STREAM(
              this->nh_->get_logger(),
              "Failed converting initial_value string to real number for the joint "
                << joint_name
                << " and state interface " << interface_info.name
                << ". Actual value of parameter: " << interface_info.initial_value
                << ". Initial value will be set to 0.0");
            throw std::invalid_argument("Failed converting initial_value string");
          }
        }
        return initial_value;
      };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();
    /******************************************************************************************************/
    double initial_pos_des = std::numeric_limits<double>::quiet_NaN();
    double initial_vel_des = std::numeric_limits<double>::quiet_NaN();
    double initial_ff = std::numeric_limits<double>::quiet_NaN();
    double initial_kp = std::numeric_limits<double>::quiet_NaN();
    double initial_kd = std::numeric_limits<double>::quiet_NaN();
    /******************************************************************************************************/

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      }
      /******************************************************************************************************/
      if (joint_info.command_interfaces[i].name == "pos_des") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t pos_des");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          "pos_des",
          &this->dataPtr->joints_[j].joint_pos_des);
        if (!std::isnan(initial_pos_des)) {
          this->dataPtr->joints_[j].joint_pos_des = initial_pos_des;
        }
      }
      if (joint_info.command_interfaces[i].name == "vel_des") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t vel_des");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          "vel_des",
          &this->dataPtr->joints_[j].joint_vel_des);
        if (!std::isnan(initial_vel_des)) {
          this->dataPtr->joints_[j].joint_vel_des = initial_vel_des;
        }
      }
      if (joint_info.command_interfaces[i].name == "ff") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t ff");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          "ff",
          &this->dataPtr->joints_[j].joint_ff);
        if (!std::isnan(initial_ff)) {
          this->dataPtr->joints_[j].joint_ff = initial_ff;
        }
      }
      if (joint_info.command_interfaces[i].name == "kp") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t kp");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          "kp",
          &this->dataPtr->joints_[j].joint_kp);
        if (!std::isnan(initial_kp)) {
          this->dataPtr->joints_[j].joint_kp = initial_kp;
        }
      }
      if (joint_info.command_interfaces[i].name == "kd") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t kd");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          "kd",
          &this->dataPtr->joints_[j].joint_kd);
        if (!std::isnan(initial_kd)) {
          this->dataPtr->joints_[j].joint_kd = initial_kd;
        }
      }
      /******************************************************************************************************/
      auto it = joint_info.command_interfaces[i].parameters.find("damping_frequency");
      if (it != joint_info.command_interfaces[i].parameters.end()) {
        double damping_frequency = stod(it->second);
        RCLCPP_INFO(
          this->nh_->get_logger(), "\t\t\t with damping_frequency %.2fs.",
          damping_frequency);
        double damping_intensity = 1.0;
        auto it2 = joint_info.command_interfaces[i].parameters.find("damping_intensity");
        if (it2 != joint_info.command_interfaces[i].parameters.end()) {
          damping_intensity = stod(it2->second);
        }
        RCLCPP_INFO(
          this->nh_->get_logger(), "\t\t\t with damping_intensity %.2fs.",
          damping_intensity);
        this->dataPtr->joints_[j].lpf =
          std::make_unique<control_toolbox::LowPassFilter<double>>(
          update_rate, damping_frequency, damping_intensity);
        this->dataPtr->joints_[j].lpf->configure();
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointPositionReset({initial_position}));
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointVelocityReset({initial_velocity}));
      }
    }
  }

  /******************************************************************************************************/
  registerSensors(hardware_info);
  /******************************************************************************************************/

  return true;
}

/******************************************************************************************************/
void QuadGzSimSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> sensor_components;
  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    sensor_components.push_back(component);
  }

  registerIMUS(sensor_components);
  registerFTS(sensor_components);
}

void QuadGzSimSystem::registerIMUS(
  const std::vector<hardware_interface::ComponentInfo> & sensor_components)
{
  this->dataPtr->ecm->Each<sim::components::Imu,
    sim::components::Name>(
    [&](const sim::Entity & entity,
    const sim::components::Imu *,
    const sim::components::Name * name) -> bool
    {
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << name->Data());
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      
      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 0},
        {"angular_velocity.y", 1},
        {"angular_velocity.z", 2},
        {"linear_acceleration.x", 0},
        {"linear_acceleration.y", 1},
        {"linear_acceleration.z", 2},
      };

      auto imu_data = std::make_shared<ImuData>();
      imu_data->name = name->Data();
      imu_data->sim_imu = entity;

      hardware_interface::ComponentInfo component;
      for (auto & comp : sensor_components) {
        if (comp.name == name->Data()) {
          component = comp;
        }
      }

      for (const auto & state_if : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_if.name);

        double* data_ptr = nullptr;
        if (state_if.name.find("covariance") != std::string::npos) {
          std::string suffix = state_if.name.substr(state_if.name.find_last_of('.') + 1);
          size_t idx = static_cast<size_t>(std::stoull(suffix));
          if (state_if.name.find("orientation_covariance") != std::string::npos)
            data_ptr = &imu_data->ori_cov[idx];
          else if (state_if.name.find("angular_velocity_covariance") != std::string::npos)
            data_ptr = &imu_data->angular_vel_cov[idx];
          else if (state_if.name.find("linear_acceleration_covariance") != std::string::npos)
            data_ptr = &imu_data->linear_acc_cov[idx];
        }
        else {
          size_t idx = interface_name_map.at(state_if.name);
          if (state_if.name.find("orientation") != std::string::npos)
            data_ptr = &imu_data->ori[idx];
          else if (state_if.name.find("angular_velocity") != std::string::npos)
            data_ptr = &imu_data->angular_vel[idx];
          else if (state_if.name.find("linear_acceleration") != std::string::npos)
            data_ptr = &imu_data->linear_acc[idx];
        }

        if (data_ptr) {
          this->dataPtr->state_interfaces_.emplace_back(
            imu_data->name,
            state_if.name,
            data_ptr);
        }
      }

      this->dataPtr->imus_.push_back(imu_data);
      return true;
    });
}

void QuadGzSimSystem::registerFTS(
  const std::vector<hardware_interface::ComponentInfo> & sensor_components)
{
  this->dataPtr->ecm->Each<sim::components::ForceTorque,
    sim::components::Name>(
    [&](const sim::Entity & entity,
    const sim::components::ForceTorque *,
    const sim::components::Name * name) -> bool
    {
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << name->Data());
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      
        // static const std::map<std::string, size_t> interface_name_map = {
        //   {"force.x", 0},
        //   {"force.y", 1},
        //   {"force.z", 2},
        //   {"torque.x", 3},
        //   {"torque.y", 4},
        //   {"torque.z", 5},
        // };      
        
      auto ft_data = std::make_shared<ForceTorqueData>();
      ft_data->name = name->Data();
      ft_data->sim_ft = entity;
      
      hardware_interface::ComponentInfo component;
      for (auto & comp : sensor_components) {
        if (comp.name == name->Data()) {
          component = comp;
        }
      }

      for (const auto & state_if : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_if.name);

        double* data_ptr = nullptr;
        if (state_if.name == "contact") data_ptr = &ft_data->contact;
          // else {
          //   size_t idx = interface_name_map.at(state_if.name);
          //   data_ptr = &ft_data->f_t[idx];
          // }
        this->dataPtr->state_interfaces_.emplace_back(
          ft_data->name,
          state_if.name,
          data_ptr);
      }
      this->dataPtr->fts_.push_back(ft_data);
      return true;
    });
}
/******************************************************************************************************/

CallbackReturn
QuadGzSimSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  /******************************************************************************************************/
  for (const auto & sensor : this->info_.sensors)
  {
    if (sensor.name == "base_imu")
    {
      try {
        double ori_cov = std::stod(sensor.parameters.at("orientation_covariance_diagonal"));
        double angular_vel_cov = std::stod(sensor.parameters.at("angular_velocity_covariance_diagonal"));
        double linear_acc_cov = std::stod(sensor.parameters.at("linear_acceleration_covariance_diagonal"));
        RCLCPP_INFO(this->nh_->get_logger(), "IMU parameter successfully read from URDF.");
        updateCovIMUS(ori_cov, angular_vel_cov, linear_acc_cov);
        RCLCPP_INFO(this->nh_->get_logger(), "IMU Covariance successfully initialized from URDF.");
      }
      catch (const std::out_of_range & e) {
        RCLCPP_ERROR(this->nh_->get_logger(), "Missing IMU parameter: %s", e.what());
        return CallbackReturn::ERROR;
      }
      catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->nh_->get_logger(), "Invalid IMU parameter value: %s", e.what());
        return CallbackReturn::ERROR;
      }
    }
  }
  /******************************************************************************************************/

  return CallbackReturn::SUCCESS;
}

/******************************************************************************************************/
void QuadGzSimSystem::updateCovIMUS(double ori_cov, double angular_vel_cov, double linear_acc_cov)
{
  for (auto &imu_data : this->dataPtr->imus_) {
    imu_data->ori_cov.fill(0.0);
    imu_data->ori_cov[0] = imu_data->ori_cov[4] = imu_data->ori_cov[8] = ori_cov;
    imu_data->angular_vel_cov.fill(0.0);
    imu_data->angular_vel_cov[0] = imu_data->angular_vel_cov[4] = imu_data->angular_vel_cov[8] = angular_vel_cov;
    imu_data->linear_acc_cov.fill(0.0);
    imu_data->linear_acc_cov[0] = imu_data->linear_acc_cov[4] = imu_data->linear_acc_cov[8] = linear_acc_cov;
  }
}
/******************************************************************************************************/

CallbackReturn QuadGzSimSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
QuadGzSimSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
QuadGzSimSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn QuadGzSimSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  /******************************************************************************************************/
  for (auto & joint : this->dataPtr->joints_) {
    joint.joint_pos_des = joint.joint_position; 
    joint.joint_vel_des = 0.0;
    joint.joint_ff      = 0.0;
    joint.joint_kp      = 50.0;
    joint.joint_kd      = 1.0;
  }
  /******************************************************************************************************/
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn QuadGzSimSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type QuadGzSimSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<sim::components::JointVelocity>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint force via joint transmitted wrench
    const auto * jointWrench =
      this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    gz::physics::Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {jointWrench->Data().force().x(),
        jointWrench->Data().force().y(), jointWrench->Data().force().z()};
    } else {  // REVOLUTE and CONTINUOUS
      force_or_torque = {jointWrench->Data().torque().x(),
        jointWrench->Data().torque().y(), jointWrench->Data().torque().z()};
    }
    // Calculate the scalar effort along the joint axis
    this->dataPtr->joints_[i].joint_effort = force_or_torque.dot(
      gz::physics::Vector3d{this->dataPtr->joints_[i].joint_axis.Xyz()[0],
        this->dataPtr->joints_[i].joint_axis.Xyz()[1],
        this->dataPtr->joints_[i].joint_axis.Xyz()[2]});
  }

  /******************************************************************************************************/
  for (auto & imu_data : this->dataPtr->imus_) {
    if (imu_data->sim_imu != sim::kNullEntity) {
      if (imu_data->topic_name.empty()) {
        auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(imu_data->sim_imu);
        if (sensorTopicComp) {
          imu_data->topic_name = sensorTopicComp->Data();
          RCLCPP_INFO_STREAM(
            this->nh_->get_logger(), "IMU " << imu_data->name << " has a topic name: " << sensorTopicComp->Data());
          this->dataPtr->node.Subscribe(imu_data->topic_name, &ImuData::OnIMU, imu_data.get());
        }
      }
    }
  }

  for (auto & ft_data : this->dataPtr->fts_) {
    if (ft_data->topic_name.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(ft_data->sim_ft);
      if (sensorTopicComp) {
        ft_data->topic_name = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "ForceTorque " << ft_data->name << " has a topic name: " << sensorTopicComp->Data());
        this->dataPtr->node.Subscribe(ft_data->topic_name, &ForceTorqueData::OnForceTorque, ft_data.get());
      }
    }
  }
  /******************************************************************************************************/

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuadGzSimSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }
    /******************************************************************************************************/
    /*
    double vel_cmd;
    if (this->dataPtr->joints_[i].lpf && this->dataPtr->joints_[i].lpf->is_configured()) {
      this->dataPtr->joints_[i].lpf->update(this->dataPtr->joints_[i].joint_velocity_cmd, vel_cmd);
    } else {
      vel_cmd = this->dataPtr->joints_[i].joint_velocity_cmd;
    }
    this->dataPtr->ecm->SetComponentData<sim::components::JointVelocityCmd>(
      this->dataPtr->joints_[i].sim_joint,
      {vel_cmd});
    */
    
    if (std::isnan(this->dataPtr->joints_[i].joint_pos_des) || 
        std::isnan(this->dataPtr->joints_[i].joint_vel_des) || 
        std::isnan(this->dataPtr->joints_[i].joint_ff) ||
        std::isnan(this->dataPtr->joints_[i].joint_kp) ||
        std::isnan(this->dataPtr->joints_[i].joint_kd)) {
      continue; 
    }

    double pos_cur = this->dataPtr->joints_[i].joint_position;
    double vel_cur = this->dataPtr->joints_[i].joint_velocity;
    double pos_des = this->dataPtr->joints_[i].joint_pos_des;
    double vel_des = this->dataPtr->joints_[i].joint_vel_des;
    double ff      = this->dataPtr->joints_[i].joint_ff;
    double kp      = this->dataPtr->joints_[i].joint_kp;
    double kd      = this->dataPtr->joints_[i].joint_kd;

    double tau_cmd = ff + kp * (pos_des - pos_cur) + kd * (vel_des - vel_cur);
    double filtered_tau;
    if (this->dataPtr->joints_[i].lpf && this->dataPtr->joints_[i].lpf->is_configured()) {
      this->dataPtr->joints_[i].lpf->update(tau_cmd, filtered_tau);
    } else {
      filtered_tau = tau_cmd;
    }

    this->dataPtr->ecm->SetComponentData<sim::components::JointForceCmd>(
      this->dataPtr->joints_[i].sim_joint,
      {filtered_tau});
    /******************************************************************************************************/
  }

  return hardware_interface::return_type::OK;
}
}  // namespace quad_robot

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  quad_robot::QuadGzSimSystem, gz_ros2_control::GazeboSimSystemInterface)
