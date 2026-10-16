#include "molly_rviz/ros_panel.hpp"

namespace molly_rviz
{

RosPanel::RosPanel(const std::string & node_name, int max_width, QWidget * parent)
: rviz_common::Panel(parent), node_name_(node_name)
{
  setMaximumWidth(max_width);
  setStyleSheet(
    "QWidget { font-size: 9pt; }"
    "QGroupBox { font-weight: 600; border: 1px solid #3a3a3a; border-radius: 4px;"
    "  margin-top: 7px; padding-top: 4px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px; }"
    "QPushButton { padding: 2px 8px; min-height: 18px; }"
    "QLabel { padding: 0px; }"
    "QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox { padding: 1px 3px; min-height: 18px; }");
}

void RosPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>(node_name_);
  setupRos();
  // rviz runs Qt's event loop, not an rclcpp executor: pump callbacks from a timer.
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &RosPanel::onSpinRos);
  spin_timer_->start(50);
}

void RosPanel::onSpinRos()
{
  rclcpp::spin_some(node_);
}

void RosPanel::setStatus(QLabel * label, const std::string & text, const std::string & color)
{
  label->setText(QString::fromStdString(text));
  label->setStyleSheet(QString::fromStdString("background-color: " + color + "; padding: 2px;"));
}

QGroupBox * RosPanel::makeGroup(const QString & title, QLayout * layout)
{
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(2);
  auto * group = new QGroupBox(title);
  group->setLayout(layout);
  return group;
}

}  // namespace molly_rviz
