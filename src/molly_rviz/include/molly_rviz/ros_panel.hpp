#pragma once

#include <string>

#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace molly_rviz
{

/// Base panel: owns a small rclcpp node pumped from the Qt event loop and shared styling.
class RosPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /// Create the panel; node_name must be unique per panel type.
  RosPanel(const std::string & node_name, int max_width, QWidget * parent = nullptr);

  /// Create the node, call setupRos() and start pumping callbacks.
  void onInitialize() override;

protected:
  /// Create publishers, subscriptions and clients on node_.
  virtual void setupRos() = 0;

  /// Show text on a label with a background colour.
  static void setStatus(QLabel * label, const std::string & text, const std::string & color);

  /// Wrap a layout in a compact titled group box.
  static QGroupBox * makeGroup(const QString & title, QLayout * layout);

  rclcpp::Node::SharedPtr node_;

private Q_SLOTS:
  /// Process pending ROS callbacks on the Qt thread.
  void onSpinRos();

private:
  std::string node_name_;
  QTimer * spin_timer_{nullptr};
};

}  // namespace molly_rviz
