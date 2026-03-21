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

#include <rviz_common/display_context.hpp>

#include "gait_panel_plugin/gait_panel.hpp"


namespace gait_panel_plugin
{

GaitPanel::GaitPanel(QWidget * parent) : rviz_common::Panel(parent)
{
    auto* layout       = new QVBoxLayout;

    auto* btn_stance   = new QPushButton("stance");
    auto* btn_trot     = new QPushButton("trot");
    auto* btn_strot    = new QPushButton("standing_trot");
    auto* btn_ftrot    = new QPushButton("flying_trot");
    auto* btn_pace     = new QPushButton("pace");
    auto* btn_space    = new QPushButton("standing_pace");
    auto* btn_dwalk    = new QPushButton("dynamic_walk");
    auto* btn_swalk    = new QPushButton("static_walk");
    auto* btn_amble    = new QPushButton("amble");
    auto* btn_lindyhop = new QPushButton("lindyhop");
    auto* btn_skipping = new QPushButton("skipping");
    auto* btn_pawup    = new QPushButton("pawup");

    layout->addWidget(btn_stance);
    layout->addWidget(btn_trot);
    layout->addWidget(btn_strot);
    layout->addWidget(btn_ftrot);
    layout->addWidget(btn_pace);
    layout->addWidget(btn_space);
    layout->addWidget(btn_dwalk);
    layout->addWidget(btn_swalk);
    layout->addWidget(btn_amble);
    layout->addWidget(btn_lindyhop);
    layout->addWidget(btn_skipping);
    layout->addWidget(btn_pawup);

    setLayout(layout);

    connect(btn_stance,   SIGNAL(clicked()), this, SLOT(sendStance()));
    connect(btn_trot,     SIGNAL(clicked()), this, SLOT(sendTrot()));
    connect(btn_strot,    SIGNAL(clicked()), this, SLOT(sendSTrot()));
    connect(btn_ftrot,    SIGNAL(clicked()), this, SLOT(sendFTrot()));
    connect(btn_pace,     SIGNAL(clicked()), this, SLOT(sendPace()));
    connect(btn_space,    SIGNAL(clicked()), this, SLOT(sendSPace()));
    connect(btn_dwalk,    SIGNAL(clicked()), this, SLOT(sendDWalk()));
    connect(btn_swalk,    SIGNAL(clicked()), this, SLOT(sendSWalk()));
    connect(btn_amble,    SIGNAL(clicked()), this, SLOT(sendAmble()));
    connect(btn_lindyhop, SIGNAL(clicked()), this, SLOT(sendLindyhop()));
    connect(btn_skipping, SIGNAL(clicked()), this, SLOT(sendSkipping()));
    connect(btn_pawup,    SIGNAL(clicked()), this, SLOT(sendPawup()));
}


void GaitPanel::onInitialize()
{
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = node_->create_publisher<std_msgs::msg::String>("quad_robot_gait_command", 10);
}

void GaitPanel::sendCommand(const std::string & cmd)
{
    auto msg = std_msgs::msg::String();
    msg.data = cmd;
    publisher_->publish(msg);
}

} // namespace gait_panel_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gait_panel_plugin::GaitPanel, rviz_common::Panel)
