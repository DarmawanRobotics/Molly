#include "molly_rviz/status_panel.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace molly_rviz
{

namespace
{
/// Format a float with a unit, or "--" when the driver reports NaN.
QString fmt(float value, int decimals, const char * unit)
{
  return std::isfinite(value) ? QString::number(value, 'f', decimals) + unit : QString("--");
}
}  // namespace

StatusPanel::StatusPanel(QWidget * parent)
: RosPanel("rviz_status_panel", 260, parent)
{
  battery_bar_ = new QProgressBar;
  battery_bar_->setRange(0, 100);
  battery_bar_->setTextVisible(false);
  battery_bar_->setFixedSize(60, 12);
  battery_label_ = new QLabel("no data");
  auto * battery_row = new QHBoxLayout;
  battery_row->addWidget(battery_bar_);
  battery_row->addWidget(battery_label_, 1);
  battery_detail_label_ = new QLabel("--");
  motor_label_ = new QLabel("motors: --");
  driver_label_ = new QLabel("driver: no diagnostics");
  driver_label_->setWordWrap(true);
  auto * robot_layout = new QVBoxLayout;
  robot_layout->addLayout(battery_row);
  robot_layout->addWidget(battery_detail_label_);
  robot_layout->addWidget(motor_label_);
  robot_layout->addWidget(driver_label_);

  cmd_vel_widget_ = new CmdVelWidget;
  cmd_vel_label_ = new QLabel("vx: 0.00\nvy: 0.00\nwz: 0.00");
  auto * cmd_vel_layout = new QHBoxLayout;
  cmd_vel_layout->addWidget(cmd_vel_widget_);
  cmd_vel_layout->addWidget(cmd_vel_label_, 1);

  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(4);
  main_layout->addWidget(makeGroup("Robot", robot_layout));
  main_layout->addWidget(makeGroup("Cmd Vel", cmd_vel_layout));
  main_layout->addStretch();
  setLayout(main_layout);
}

void StatusPanel::setupRos()
{
  // Relative names: remap from the rviz2 command line, e.g. -r battery:=/robot/battery
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", 10, std::bind(&StatusPanel::batteryCallback, this, std::placeholders::_1));
  motor_temp_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "motor_temperature", 10,
    std::bind(&StatusPanel::motorTemperatureCallback, this, std::placeholders::_1));
  diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10, std::bind(&StatusPanel::diagnosticsCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&StatusPanel::cmdVelCallback, this, std::placeholders::_1));
}

void StatusPanel::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  battery_detail_label_->setText(
    fmt(msg->voltage, 1, " V") + "  " + fmt(msg->current, 1, " A") + "  " +
    fmt(msg->temperature, 0, " °C"));
  if (!std::isfinite(msg->percentage)) {
    battery_label_->setText("unknown");
    return;
  }
  const int percent = static_cast<int>(std::round(msg->percentage * 100.0f));
  battery_bar_->setValue(std::clamp(percent, 0, 100));
  battery_label_->setText(QString("%1%").arg(percent));
  std::string color = "#88ff88";
  if (msg->percentage < 0.20f) {
    color = "#ff4444";
  } else if (msg->percentage < 0.40f) {
    color = "#ffaa00";
  }
  battery_bar_->setStyleSheet(
    QString::fromStdString("QProgressBar::chunk { background-color: " + color + "; }"));
}

void StatusPanel::motorTemperatureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.empty()) {
    return;
  }
  const auto it = std::max_element(msg->data.begin(), msg->data.end());
  const float hottest = *it;
  const auto index = static_cast<int>(std::distance(msg->data.begin(), it));
  std::string color = hottest > 70.0f ? "#ff8888" : hottest > 55.0f ? "#ffcc00" : "#88ff88";
  setStatus(
    motor_label_, "motors max " + std::to_string(static_cast<int>(hottest)) + " °C (#" +
    std::to_string(index) + ")", color);
}

void StatusPanel::diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  if (msg->status.empty()) {
    return;
  }
  const auto & status = msg->status.front();
  std::string mode;
  for (const auto & kv : status.values) {
    if (kv.key == "control_mode" || kv.key == "function_mode") {
      mode += (mode.empty() ? "" : " / ") + kv.value;
    }
  }
  std::string color = "#88ff88";
  if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    color = "#ffcc00";
  } else if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    color = "#ff8888";
  }
  setStatus(driver_label_, status.message + (mode.empty() ? "" : " | " + mode), color);
}

void StatusPanel::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_widget_->setValues(msg->linear.x, msg->linear.y, msg->angular.z);
  cmd_vel_label_->setText(
    QString("vx: %1 m/s\nvy: %2 m/s\nwz: %3 rad/s")
    .arg(msg->linear.x, 0, 'f', 2)
    .arg(msg->linear.y, 0, 'f', 2)
    .arg(msg->angular.z, 0, 'f', 2));
}

}  // namespace molly_rviz

PLUGINLIB_EXPORT_CLASS(molly_rviz::StatusPanel, rviz_common::Panel)
