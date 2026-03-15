/******************************************************************************
Copyright (c) 2026, Yuxin Li. All rights reserved.
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

using namespace ocs2;

namespace {
scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState(12);
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  return std::max(rotationTime, displacementTime);
}

class LeggedRobotGoalPublisher {
 public:
  LeggedRobotGoalPublisher(rclcpp::Node::SharedPtr node, const std::string& robotName)
      : node_(node) {

    rosReferencePublisherPtr_.reset(new TargetTrajectoriesRosPublisher(node, robotName));

    goalSubscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, std::bind(&LeggedRobotGoalPublisher::goalCallback, this, std::placeholders::_1));

    observationSubscriber_ = node->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName + "_mpc_observation", 1, [&](const ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
          latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        });
    
    latestObservation_.time = -1.0;
  }

 private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (latestObservation_.time < 0) {
        RCLCPP_WARN(node_->get_logger(), "Waiting for observation...");
        return;
    }

    const vector_t currentPose = latestObservation_.state.segment<6>(6);

    vector_t targetPose = currentPose;
    targetPose(0) = msg->pose.position.x;
    targetPose(1) = msg->pose.position.y;
    targetPose(2) = comHeight;

    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    targetPose(3) = yaw;

    const scalar_t reachingTime = latestObservation_.time + estimateTimeToTarget(targetPose - currentPose);
    
    TargetTrajectories targetTrajectories;
    targetTrajectories.timeTrajectory = {latestObservation_.time, reachingTime};
    
    vector_t targetState(latestObservation_.state.size());
    targetState << vector_t::Zero(6), targetPose, defaultJointState;
    
    targetTrajectories.stateTrajectory = {latestObservation_.state, targetState};
    targetTrajectories.inputTrajectory = {vector_t::Zero(latestObservation_.input.size()), 
                                          vector_t::Zero(latestObservation_.input.size())};

    rosReferencePublisherPtr_->publishTargetTrajectories(targetTrajectories);
    RCLCPP_INFO(node_->get_logger(), "New goal published from RViz.");
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<TargetTrajectoriesRosPublisher> rosReferencePublisherPtr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscriber_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
  SystemObservation latestObservation_;
};

int main(int argc, char* argv[]) {
  const std::string robotName = "quad_robot";

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(robotName + "_rviz_target");

  const std::string referenceFile =
      node->declare_parameter<std::string>("referenceFile", "");
  if (referenceFile.empty()) {
    throw std::runtime_error(
        "[LeggedRobotPoseCommandNode] Parameter 'referenceFile' is required.");
  }

  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState",
                            defaultJointState);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity",
                            targetRotationVelocity);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity",
                            targetDisplacementVelocity);

  LeggedRobotGoalPublisher goalPublisher(node, robotName);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
