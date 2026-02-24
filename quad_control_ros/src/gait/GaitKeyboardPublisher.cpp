/******************************************************************************
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

#include "quad_control_ros/gait/GaitKeyboardPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include <algorithm>
#include <ocs2_msgs/msg/mode_schedule.hpp>

#include "quad_control_ros/gait/ModeSequenceTemplateRos.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitKeyboardPublisher::GaitKeyboardPublisher(
    const rclcpp::Node::SharedPtr& node, const std::string& gaitFile,
    const std::string& robotName, bool verbose) {
  RCLCPP_INFO_STREAM(node->get_logger(),
                     robotName + "_mpc_mode_schedule node is setting up ...");
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ =
      node->create_publisher<ocs2_msgs::msg::ModeSchedule>(
          robotName + "_mpc_mode_schedule", 1);
  
  gaitSubscriber_ = node->create_subscription<std_msgs::msg::String>(
      robotName + "_gait_command", 10, std::bind(&GaitKeyboardPublisher::gaitCallback, this, std::placeholders::_1));

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert(
        {gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     robotName + "_mpc_mode_schedule command node is ready.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::gaitCallback(const std_msgs::msg::String::SharedPtr msg) {
  // lower case transform
  auto gaitCommand = msg->data;
  std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(),
                 ::tolower);
  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }
  try {
    ModeSequenceTemplate ModeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(
        createModeSequenceTemplateMsg(ModeSequenceTemplate));
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::printGaitList(
    const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

}  // namespace legged_robot
}  // end of namespace ocs2
