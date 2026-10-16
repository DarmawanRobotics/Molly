#include "molly_rviz/tag_panel.hpp"

#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace molly_rviz
{

TagPanel::TagPanel(QWidget * parent)
: RosPanel("rviz_tag_panel", 300, parent)
{
  record_button_ = new QPushButton("Record Tag Poses");
  record_button_->setToolTip(
    "Average the map pose of every visible tag for a few seconds and save it to the tag file. "
    "Do this while teaching, with the robot standing still.");
  localize_button_ = new QPushButton("Localize from Tag");
  localization_label_ = new QLabel("not localized");
  localization_label_->setWordWrap(true);
  result_label_ = new QLabel("Stand still in view of a tag, then record.");
  result_label_->setWordWrap(true);
  result_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  auto * layout = new QVBoxLayout;
  layout->addWidget(record_button_);
  layout->addWidget(localize_button_);
  layout->addWidget(localization_label_);
  layout->addWidget(result_label_);

  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->addWidget(makeGroup("AprilTags", layout));
  main_layout->addStretch();
  setLayout(main_layout);

  connect(record_button_, &QPushButton::clicked, this, &TagPanel::onRecordTags);
  connect(localize_button_, &QPushButton::clicked, this, &TagPanel::onLocalize);
}

void TagPanel::setupRos()
{
  record_client_ = node_->create_client<std_srvs::srv::Trigger>("localization/record_tags");
  localize_client_ = node_->create_client<std_srvs::srv::Trigger>("localization/start");
  localization_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "localization/status", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const bool localized = msg->data != "none";
      setStatus(
        localization_label_, localized ? "localized: " + msg->data : "not localized",
        localized ? "#88ff88" : "#ffcc00");
    });
}

void TagPanel::onRecordTags()
{
  callTrigger(record_client_, "recording tags, keep the robot still...");
}

void TagPanel::onLocalize()
{
  callTrigger(localize_client_, "localizing...");
}

void TagPanel::callTrigger(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const std::string & busy_text)
{
  if (!client->service_is_ready()) {
    setStatus(result_label_, std::string(client->get_service_name()) + " not available", "#ffcc00");
    return;
  }
  setStatus(result_label_, busy_text, "#ffffff");
  record_button_->setEnabled(false);
  localize_button_->setEnabled(false);
  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      setStatus(result_label_, response->message, response->success ? "#88ff88" : "#ff8888");
      record_button_->setEnabled(true);
      localize_button_->setEnabled(true);
    });
}

}  // namespace molly_rviz

PLUGINLIB_EXPORT_CLASS(molly_rviz::TagPanel, rviz_common::Panel)
