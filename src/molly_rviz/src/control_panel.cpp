#include "molly_rviz/control_panel.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace molly_rviz
{

ControlPanel::ControlPanel(QWidget * parent)
: RosPanel("rviz_mission_panel", 260, parent)
{
  // Localization
  localization_button_ = new QPushButton("Localize from Tag");
  localization_status_label_ = new QLabel("not localized");
  localization_status_label_->setWordWrap(true);
  auto * localization_layout = new QVBoxLayout;
  localization_layout->addWidget(localization_button_);
  localization_layout->addWidget(localization_status_label_);

  // Route recording
  record_button_ = new QPushButton("Start Recording");
  dwell_spinbox_ = new QDoubleSpinBox;
  dwell_spinbox_->setRange(0.5, 600.0);
  dwell_spinbox_->setValue(10.0);
  dwell_spinbox_->setSuffix(" s");
  mark_stop_point_button_ = new QPushButton("Mark Stop Point");
  recording_status_label_ = new QLabel("not recording");
  recording_status_label_->setWordWrap(true);
  auto * dwell_row = new QHBoxLayout;
  dwell_row->addWidget(new QLabel("Dwell:"));
  dwell_row->addWidget(dwell_spinbox_);
  dwell_row->addWidget(mark_stop_point_button_);
  auto * recording_layout = new QVBoxLayout;
  recording_layout->addWidget(record_button_);
  recording_layout->addLayout(dwell_row);
  recording_layout->addWidget(recording_status_label_);

  // Route loading
  waypoints_file_edit_ = new QLineEdit("/home/robot/dev/Molly/map/example/example_waypoint.csv");
  browse_button_ = new QPushButton("...");
  load_button_ = new QPushButton("Load Route");
  load_status_label_ = new QLabel("no route loaded");
  load_status_label_->setWordWrap(true);
  auto * file_row = new QHBoxLayout;
  file_row->addWidget(waypoints_file_edit_, 1);
  file_row->addWidget(browse_button_);
  auto * load_layout = new QVBoxLayout;
  load_layout->addLayout(file_row);
  load_layout->addWidget(load_button_);
  load_layout->addWidget(load_status_label_);

  // Navigation
  nav_button_ = new QPushButton("Start Navigation");
  pause_button_ = new QPushButton("Pause");
  pause_button_->setEnabled(false);
  nav_status_label_ = new QLabel("NAV_INACTIVE");
  nav_status_label_->setWordWrap(true);
  mission_label_ = new QLabel("no mission status");
  mission_label_->setWordWrap(true);
  auto * nav_buttons = new QHBoxLayout;
  nav_buttons->addWidget(nav_button_);
  nav_buttons->addWidget(pause_button_);
  auto * nav_layout = new QVBoxLayout;
  nav_layout->addLayout(nav_buttons);
  nav_layout->addWidget(nav_status_label_);
  nav_layout->addWidget(mission_label_);

  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(4);
  main_layout->addWidget(makeGroup("Localization", localization_layout));
  main_layout->addWidget(makeGroup("Route Recording", recording_layout));
  main_layout->addWidget(makeGroup("Route", load_layout));
  main_layout->addWidget(makeGroup("Navigation", nav_layout));
  main_layout->addStretch();
  setLayout(main_layout);

  connect(localization_button_, &QPushButton::clicked, this, &ControlPanel::onTriggerLocalization);
  connect(record_button_, &QPushButton::clicked, this, &ControlPanel::onToggleRecording);
  connect(mark_stop_point_button_, &QPushButton::clicked, this, &ControlPanel::onMarkStopPoint);
  connect(browse_button_, &QPushButton::clicked, this, &ControlPanel::onBrowseWaypointsFile);
  connect(load_button_, &QPushButton::clicked, this, &ControlPanel::onLoadPath);
  connect(nav_button_, &QPushButton::clicked, this, &ControlPanel::onToggleNavigation);
  connect(pause_button_, &QPushButton::clicked, this, &ControlPanel::onTogglePause);
}

void ControlPanel::setupRos()
{
  localization_client_ = node_->create_client<std_srvs::srv::Trigger>("localization/start");
  record_client_ = node_->create_client<std_srvs::srv::SetBool>("route/record");
  mark_stop_point_client_ =
    node_->create_client<molly_interfaces::srv::MarkStopPoint>("route/mark_stop_point");
  load_path_client_ = node_->create_client<molly_interfaces::srv::LoadPath>("navigation/load_path");
  nav_client_ = node_->create_client<std_srvs::srv::SetBool>("navigation/start_nav");
  pause_client_ = node_->create_client<std_srvs::srv::SetBool>("navigation/pause");

  auto latched = rclcpp::QoS(1).transient_local();
  localization_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "localization/status", latched,
    std::bind(&ControlPanel::localizationStatusCallback, this, std::placeholders::_1));
  nav_status_sub_ = node_->create_subscription<molly_interfaces::msg::NavigationStatus>(
    "navigation/status", 10,
    std::bind(&ControlPanel::navStatusCallback, this, std::placeholders::_1));
  mission_status_sub_ = node_->create_subscription<molly_interfaces::msg::MissionStatus>(
    "navigation/mission_status", 10,
    std::bind(&ControlPanel::missionStatusCallback, this, std::placeholders::_1));
}

void ControlPanel::onTriggerLocalization()
{
  if (!localization_client_->service_is_ready()) {
    setStatus(localization_status_label_, "localization/start not available", "#ffcc00");
    return;
  }
  setStatus(localization_status_label_, "calling...", "#ffffff");
  localization_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      setStatus(
        localization_status_label_, response->message, response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onToggleRecording()
{
  if (!record_client_->service_is_ready()) {
    setStatus(recording_status_label_, "route/record not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_recording_;
  record_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        is_recording_ = !is_recording_;
        record_button_->setText(is_recording_ ? "Stop Recording" : "Start Recording");
      }
      setStatus(
        recording_status_label_, response->message, response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onMarkStopPoint()
{
  if (!mark_stop_point_client_->service_is_ready()) {
    setStatus(recording_status_label_, "route/mark_stop_point not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<molly_interfaces::srv::MarkStopPoint::Request>();
  request->dwell_sec = static_cast<float>(dwell_spinbox_->value());
  mark_stop_point_client_->async_send_request(
    request,
    [this](rclcpp::Client<molly_interfaces::srv::MarkStopPoint>::SharedFuture future) {
      auto response = future.get();
      setStatus(
        recording_status_label_, response->message, response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onBrowseWaypointsFile()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, "Select route CSV", waypoints_file_edit_->text(), "CSV files (*.csv)");
  if (!filename.isEmpty()) {
    waypoints_file_edit_->setText(filename);
  }
}

void ControlPanel::onLoadPath()
{
  if (!load_path_client_->service_is_ready()) {
    setStatus(load_status_label_, "navigation/load_path not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<molly_interfaces::srv::LoadPath::Request>();
  request->waypoints_file = waypoints_file_edit_->text().toStdString();
  setStatus(load_status_label_, "loading...", "#ffffff");
  load_path_client_->async_send_request(
    request, [this](rclcpp::Client<molly_interfaces::srv::LoadPath>::SharedFuture future) {
      auto response = future.get();
      setStatus(load_status_label_, response->message, response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onToggleNavigation()
{
  if (!nav_client_->service_is_ready()) {
    setStatus(nav_status_label_, "navigation/start_nav not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_navigating_;
  nav_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      auto response = future.get();
      if (!response->success) {
        setStatus(nav_status_label_, response->message, "#ff8888");
      }
    });
}

void ControlPanel::onTogglePause()
{
  if (!pause_client_->service_is_ready()) {
    setStatus(nav_status_label_, "navigation/pause not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_paused_;
  pause_client_->async_send_request(
    request, [](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture) {});
}

void ControlPanel::navStatusCallback(const molly_interfaces::msg::NavigationStatus::SharedPtr msg)
{
  const std::string & s = msg->state;
  std::string color = "#cccccc";
  if (s == "FOLLOWING") {
    color = "#88ff88";
  } else if (s == "GOAL_REACHED") {
    color = "#88bbff";
  } else if (s == "APPROACHING" || s == "DWELLING" || s == "PAUSED") {
    color = "#ffff88";
  } else if (s == "NO_PATH" || s == "BLOCKED") {
    color = "#ffaa55";
  } else if (
    s == "EMERGENCY_STOP" || s == "TF_UNAVAILABLE" || s == "AVOIDANCE_STALE" || s == "OFF_PATH")
  {
    color = "#ff8888";
  }

  // Buttons follow the follower state, so they stay right when it is driven from elsewhere.
  is_navigating_ = s != "NAV_INACTIVE";
  is_paused_ = s == "PAUSED";
  nav_button_->setText(is_navigating_ ? "Stop Navigation" : "Start Navigation");
  pause_button_->setEnabled(is_navigating_);
  pause_button_->setText(is_paused_ ? "Resume" : "Pause");
  setStatus(nav_status_label_, s + ": " + msg->message, color);
}

void ControlPanel::missionStatusCallback(const molly_interfaces::msg::MissionStatus::SharedPtr msg)
{
  mission_label_->setText(QString::fromStdString(msg->active_behavior));
}

void ControlPanel::localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const bool localized = msg->data != "none";
  setStatus(
    localization_status_label_, localized ? "localized: " + msg->data : "not localized",
    localized ? "#88ff88" : "#ffcc00");
}

}  // namespace molly_rviz

PLUGINLIB_EXPORT_CLASS(molly_rviz::ControlPanel, rviz_common::Panel)
