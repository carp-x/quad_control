#include <rviz_common/display_context.hpp>

#include "gait_panel_plugin/gait_panel.hpp"


namespace gait_panel_plugin
{

GaitPanel::GaitPanel(QWidget * parent) : rviz_common::Panel(parent)
{
    auto* layout = new QVBoxLayout;
    auto* btn_trot = new QPushButton("Trot");

    layout->addWidget(btn_trot);
    setLayout(layout);

    connect(btn_trot, SIGNAL(clicked()), this, SLOT(sendTrot()));
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
