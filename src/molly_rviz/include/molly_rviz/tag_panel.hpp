#pragma once

#include <QLabel>
#include <QPushButton>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "molly_rviz/ros_panel.hpp"

namespace molly_rviz
{

/// AprilTag tools: record tag poses in the map frame and localize from a visible tag.
class TagPanel : public RosPanel
{
  Q_OBJECT

public:
  explicit TagPanel(QWidget * parent = nullptr);

protected:
  void setupRos() override;

private Q_SLOTS:
  void onRecordTags();
  void onLocalize();

private:
  /// Call a Trigger service and show its response on result_label_.
  void callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & busy_text);

  QPushButton * record_button_;
  QPushButton * localize_button_;
  QLabel * localization_label_;
  QLabel * result_label_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr localize_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
};

}  // namespace molly_rviz
