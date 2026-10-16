#include "molly_rviz/cmd_vel_widget.hpp"

#include <QPainter>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

namespace molly_rviz
{

CmdVelWidget::CmdVelWidget(QWidget * parent)
: QWidget(parent)
{
  setFixedSize(96, 96);
}

void CmdVelWidget::setValues(double linear_x, double linear_y, double angular_z)
{
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_z_ = angular_z;
  update();
}

void CmdVelWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int w = width();
  const int h = height();
  const int cx = w / 2;
  const int cy = h / 2;
  const int radius = std::min(w, h) / 2 - 4;

  // Background disc
  painter.setPen(QPen(QColor("#555555"), 1));
  painter.setBrush(QColor("#202020"));
  painter.drawEllipse(QPoint(cx, cy), radius, radius);

  // Crosshair reference lines
  painter.setPen(QPen(QColor("#444444"), 1, Qt::DashLine));
  painter.drawLine(cx - radius, cy, cx + radius, cy);
  painter.drawLine(cx, cy - radius, cx, cy + radius);

  // angular.z -> arc sweeping left/right from top, proportional to |wz|
  const double ang_ratio = std::clamp(angular_z_ / max_angular_, -1.0, 1.0);
  const int span_angle = static_cast<int>(-ang_ratio * 120.0 * 16);  // Qt units: 1/16 deg, CCW+
  painter.setPen(QPen(QColor("#ffaa00"), 3));
  QRectF arc_rect(cx - radius + 2, cy - radius + 2, (radius - 2) * 2.0, (radius - 2) * 2.0);
  painter.drawArc(arc_rect, 90 * 16, span_angle);

  // linear.x/y -> dot position. Robot convention: +x forward (up), +y left (left).
  const double lx_ratio = std::clamp(linear_x_ / max_linear_, -1.0, 1.0);
  const double ly_ratio = std::clamp(linear_y_ / max_linear_, -1.0, 1.0);
  const int dot_x = cx - static_cast<int>(ly_ratio * (radius - 8));
  const int dot_y = cy - static_cast<int>(lx_ratio * (radius - 8));

  painter.setPen(QPen(QColor("#66ccff"), 2));
  painter.drawLine(cx, cy, dot_x, dot_y);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#66ccff"));
  painter.drawEllipse(QPoint(dot_x, dot_y), 5, 5);
}

}  