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
  void sendStart() { sendCommand("start"); }
  void sendStop() { sendCommand("stop"); }
  void sendTrot() { sendCommand("trot"); }

protected:
  void sendCommand(const std::string & cmd);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
};

} // namespace gait_panel_plugin
