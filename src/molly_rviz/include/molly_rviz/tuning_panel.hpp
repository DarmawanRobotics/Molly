#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>

#include "molly_rviz/ros_panel.hpp"

namespace molly_rviz
{

/// Live parameter editor for navigation nodes with controller/algorithm selection and YAML export.
class TuningPanel : public RosPanel
{
  Q_OBJECT

public:
  explicit TuningPanel(QWidget * parent = nullptr);

protected:
  void setupRos() override;

private Q_SLOTS:
  void onLoad();
  void onApply();
  void onSaveYaml();
  void onSelectorActivated(int index);

private:
  /// Rebuild the editor form from params_.
  void render();
  /// Create an editor widget for one parameter.
  QWidget * makeEditor(const std::string & name, const rclcpp::Parameter & param);
  /// Read the value currently shown in a parameter's editor.
  rclcpp::Parameter editorValue(const std::string & name) const;
  /// Record an edit and apply it right away in live mode.
  void markDirty(const std::string & name);
  /// Send parameters to the target node and report the result.
  void sendParameters(const std::vector<rclcpp::Parameter> & params, bool rerender);
  /// Name of the parameter that selects the algorithm ("controller", "algorithm" or "").
  std::string selectorName() const;

  QComboBox * node_combo_;
  QComboBox * selector_combo_;
  QLabel * selector_label_;
  QCheckBox * live_check_;
  QCheckBox * show_all_check_;
  QPushButton * load_button_;
  QPushButton * apply_button_;
  QPushButton * save_button_;
  QScrollArea * scroll_;
  QLabel * status_label_;

  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  std::string target_;
  std::map<std::string, rclcpp::Parameter> params_;
  std::map<std::string, QWidget *> editors_;
  std::set<std::string> dirty_;
  bool rendering_{false};
};

}  // namespace molly_rviz
