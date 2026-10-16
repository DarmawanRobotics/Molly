#pragma once

#include <memory>
#include <string>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <molly_interfaces/msg/mission_status.hpp>
#include <molly_interfaces/msg/navigation_status.hpp>
#include <molly_interfaces/srv/load_path.hpp>
#include <molly_interfaces/srv/mark_stop_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "molly_rviz/ros_panel.hpp"

namespace molly_rviz
{

/// Mission panel: localization, route recording, route loading, start/stop/pause navigation.
class ControlPanel : public RosPanel
{
  Q_OBJECT

public:
  explicit ControlPanel(QWidget * parent = nullptr);

protected:
  void setupRos() override;

private Q_SLOTS:
  void onTriggerLocalization();
  void onToggleRecording();
  void onMarkStopPoint();
  void onBrowseWaypointsFile();
  void onLoadPath();
  void onToggleNavigation();
  void onTogglePause();

private:
  void navStatusCallback(const molly_interfaces::msg::NavigationStatus::SharedPtr msg);
  void missionStatusCallback(const molly_interfaces::msg::MissionStatus::SharedPtr msg);
  void localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg);

  // Localization
  QPushButton * localization_button_;
  QLabel * localization_status_label_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr localization_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;

  // Route recording
  QPushButton * record_button_;
  QDoubleSpinBox * dwell_spinbox_;
  QPushButton * mark_stop_point_button_;
  QLabel * recording_status_label_;
  bool is_recording_{false};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr record_client_;
  rclcpp::Client<molly_interfaces::srv::MarkStopPoint>::SharedPtr mark_stop_point_client_;

  // Route loading
  QLineEdit * waypoints_file_edit_;
  QPushButton * browse_button_;
  QPushButton * load_button_;
  QLabel * load_status_label_;
  rclcpp::Client<molly_interfaces::srv::LoadPath>::SharedPtr load_path_client_;

  // Navigation
  QPushButton * nav_button_;
  QPushButton * pause_button_;
  QLabel * nav_status_label_;
  QLabel * mission_label_;
  bool is_navigating_{false};
  bool is_paused_{false};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr nav_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client_;
  rclcpp::Subscription<molly_interfaces::msg::NavigationStatus>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<molly_interfaces::msg::MissionStatus>::SharedPtr mission_status_sub_;
};

}  // namespace molly_rviz
