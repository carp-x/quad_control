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
