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

#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QPushButton>
#include <QVBoxLayout>


namespace gait_panel_plugin
{

class GaitPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  GaitPanel(QWidget * parent = nullptr);
  virtual void onInitialize() override;

protected Q_SLOTS:
  void sendStance()    { sendCommand("stance"); }
  void sendTrot()      { sendCommand("trot"); }
  void sendSTrot()     { sendCommand("standing_trot"); }
  void sendFTrot()     { sendCommand("flying_trot"); }
  void sendPace()      { sendCommand("pace"); }
  void sendSPace()     { sendCommand("standing_pace"); }
  void sendDWalk()     { sendCommand("dynamic_walk"); }
  void sendSWalk()     { sendCommand("static_walk"); }
  void sendAmble()     { sendCommand("amble"); }
  void sendLindyhop()  { sendCommand("lindyhop"); }
  void sendSkipping()  { sendCommand("skipping"); }
  void sendPawup()     { sendCommand("pawup"); }

protected:
  void sendCommand(const std::string & cmd);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
};

} // namespace gait_panel_plugin
