#pragma once

#include <QLabel>
#include <QProgressBar>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "molly_rviz/cmd_vel_widget.hpp"
#include "molly_rviz/ros_panel.hpp"

namespace molly_rviz
{

/// Robot status panel: battery, temperatures, driver diagnostics and the commanded velocity.
class StatusPanel : public RosPanel
{
  Q_OBJECT

public:
  explicit StatusPanel(QWidget * parent = nullptr);

protected:
  void setupRos() override;

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void motorTemperatureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  QProgressBar * battery_bar_;
  QLabel * battery_label_;
  QLabel * battery_detail_label_;
  QLabel * motor_label_;
  QLabel * driver_label_;
  CmdVelWidget * cmd_vel_widget_;
  QLabel * cmd_vel_label_;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_temp_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace molly_rviz
