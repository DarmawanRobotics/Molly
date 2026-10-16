#pragma once

#include <QWidget>

namespace molly_rviz
{
// Compact radar-style readout for a Twist: linear.x/y as a dot position,
// angular.z as an arc. Purely a dumb display widget -- feed it values via
// setValues(), it repaints itself.
class CmdVelWidget : public QWidget
{
  Q_OBJECT

public:
  explicit CmdVelWidget(QWidget * parent = nullptr);

  void setValues(double linear_x, double linear_y, double angular_z);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  double linear_x_ = 0.0;
  double linear_y_ = 0.0;
  double angular_z_ = 0.0;

  // Full-scale values used to map m/s and rad/s onto the widget's radius.
  // Tune these to whatever your robot's typical max speeds are.
  double max_linear_ = 1.0;   // m/s
  double max_angular_ = 2.0;  // rad/s
};

}  // namespace molly_rviz

