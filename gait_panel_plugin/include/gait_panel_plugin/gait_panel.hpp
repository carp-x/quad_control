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
