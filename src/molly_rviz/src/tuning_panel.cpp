#include "molly_rviz/tuning_panel.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>

#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QSpinBox>
#include <QStringList>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace molly_rviz
{

namespace
{
using ParamType = rclcpp::ParameterType;

/// Options of the parameters that select an algorithm; their groups are hidden unless selected.
const std::map<std::string, std::vector<std::string>> kSelectors = {
  {"controller", {"holonomic", "pure_pursuit", "pid", "mppi", "lqr", "stanley"}},
  {"algorithm", {"braitenberg", "vfh", "potential_field", "follow_gap"}},
};

const QStringList kNodes = {
  "/path_follower_node", "/obstacle_avoidance_node", "/path_loader_node", "/lidar_sector_node",
  "/localization_node"};

/// Format a double so it reads back as a double from YAML (always has a '.' or exponent).
std::string yamlDouble(double v)
{
  std::string s = QString::number(v, 'g', 12).toStdString();
  if (s.find_first_of(".einn") == std::string::npos) {
    s += ".0";
  }
  return s;
}

/// Quote a string for YAML.
std::string yamlString(const std::string & s)
{
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out + "\"";
}

/// Serialise one parameter value for a ROS 2 parameter YAML file.
std::string yamlValue(const rclcpp::Parameter & p)
{
  switch (p.get_type()) {
    case ParamType::PARAMETER_BOOL:
      return p.as_bool() ? "true" : "false";
    case ParamType::PARAMETER_INTEGER:
      return std::to_string(p.as_int());
    case ParamType::PARAMETER_DOUBLE:
      return yamlDouble(p.as_double());
    case ParamType::PARAMETER_STRING:
      return yamlString(p.as_string());
    case ParamType::PARAMETER_STRING_ARRAY: {
        std::string out = "[";
        for (const auto & s : p.as_string_array()) {
          out += (out.size() > 1 ? ", " : "") + yamlString(s);
        }
        return out + "]";
      }
    default:
      return p.value_to_string();
  }
}

/// Spin-box step that suits the magnitude of a value.
double stepFor(double v)
{
  const double magnitude = std::abs(v);
  return magnitude >= 1.0 ? 0.1 : magnitude >= 0.1 ? 0.01 : 0.001;
}
}  // namespace

TuningPanel::TuningPanel(QWidget * parent)
: RosPanel("rviz_tuning_panel", 360, parent)
{
  node_combo_ = new QComboBox;
  node_combo_->setEditable(true);
  node_combo_->addItems(kNodes);
  load_button_ = new QPushButton("Load");
  auto * node_row = new QHBoxLayout;
  node_row->addWidget(node_combo_, 1);
  node_row->addWidget(load_button_);

  selector_label_ = new QLabel("algorithm");
  selector_combo_ = new QComboBox;
  selector_combo_->setEnabled(false);
  auto * selector_row = new QHBoxLayout;
  selector_row->addWidget(selector_label_);
  selector_row->addWidget(selector_combo_, 1);

  live_check_ = new QCheckBox("Apply on change");
  live_check_->setChecked(true);
  show_all_check_ = new QCheckBox("Show all groups");
  auto * option_row = new QHBoxLayout;
  option_row->addWidget(live_check_);
  option_row->addWidget(show_all_check_);

  scroll_ = new QScrollArea;
  scroll_->setWidgetResizable(true);
  scroll_->setMinimumHeight(250);

  apply_button_ = new QPushButton("Apply");
  save_button_ = new QPushButton("Save YAML...");
  auto * action_row = new QHBoxLayout;
  action_row->addWidget(apply_button_);
  action_row->addWidget(save_button_);

  status_label_ = new QLabel("pick a node and press Load");
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout;
  layout->addLayout(node_row);
  layout->addLayout(selector_row);
  layout->addLayout(option_row);
  layout->addWidget(scroll_, 1);
  layout->addLayout(action_row);
  layout->addWidget(status_label_);

  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->addWidget(makeGroup("Parameter Tuning", layout), 1);
  setLayout(main_layout);

  connect(load_button_, &QPushButton::clicked, this, &TuningPanel::onLoad);
  connect(apply_button_, &QPushButton::clicked, this, &TuningPanel::onApply);
  connect(save_button_, &QPushButton::clicked, this, &TuningPanel::onSaveYaml);
  connect(
    selector_combo_, QOverload<int>::of(&QComboBox::activated), this,
    &TuningPanel::onSelectorActivated);
  connect(show_all_check_, &QCheckBox::toggled, this, [this](bool) {render();});
}

void TuningPanel::setupRos() {}

void TuningPanel::onLoad()
{
  target_ = node_combo_->currentText().trimmed().toStdString();
  if (target_.empty()) {
    return;
  }
  if (target_.front() != '/') {
    target_ = "/" + target_;
  }
  client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, target_);
  if (!client_->wait_for_service(std::chrono::milliseconds(1000))) {
    setStatus(status_label_, target_ + " is not running", "#ffcc00");
    return;
  }
  setStatus(status_label_, "loading " + target_ + "...", "#ffffff");
  client_->list_parameters(
    {}, 0, [this](std::shared_future<rcl_interfaces::msg::ListParametersResult> listed) {
      std::vector<std::string> names;
      for (const auto & name : listed.get().names) {
        if (name.rfind("qos_overrides", 0) != 0 && name != "use_sim_time") {
          names.push_back(name);
        }
      }
      client_->get_parameters(
        names, [this](std::shared_future<std::vector<rclcpp::Parameter>> values) {
          params_.clear();
          for (const auto & p : values.get()) {
            params_.emplace(p.get_name(), p);
          }
          dirty_.clear();
          render();
          setStatus(
            status_label_, "loaded " + std::to_string(params_.size()) + " parameters",
            "#88ff88");
        });
    });
}

std::string TuningPanel::selectorName() const
{
  for (const auto & [name, options] : kSelectors) {
    (void)options;
    if (params_.count(name)) {
      return name;
    }
  }
  return "";
}

void TuningPanel::render()
{
  rendering_ = true;
  editors_.clear();

  const std::string selector = selectorName();
  const std::string selected = selector.empty() ? "" : params_.at(selector).as_string();
  selector_combo_->clear();
  selector_combo_->setEnabled(!selector.empty());
  selector_label_->setText(selector.empty() ? "algorithm" : QString::fromStdString(selector));
  std::set<std::string> algorithm_groups;
  if (!selector.empty()) {
    for (const auto & option : kSelectors.at(selector)) {
      selector_combo_->addItem(QString::fromStdString(option));
      algorithm_groups.insert(option);
    }
    selector_combo_->setCurrentText(QString::fromStdString(selected));
  }

  // Group by the prefix before the first dot: general first, then the selected algorithm.
  std::map<std::string, std::vector<std::string>> groups;
  for (const auto & [name, param] : params_) {
    (void)param;
    if (name == selector) {
      continue;
    }
    const auto dot = name.find('.');
    groups[dot == std::string::npos ? "general" : name.substr(0, dot)].push_back(name);
  }
  std::vector<std::string> order;
  if (groups.count("general")) {
    order.push_back("general");
  }
  if (groups.count(selected)) {
    order.push_back(selected);
  }
  for (const auto & [group, names] : groups) {
    (void)names;
    if (group == "general" || group == selected) {
      continue;
    }
    if (algorithm_groups.count(group) && !show_all_check_->isChecked()) {
      continue;
    }
    order.push_back(group);
  }

  auto * content = new QWidget;
  auto * vbox = new QVBoxLayout(content);
  vbox->setContentsMargins(0, 0, 0, 0);
  for (const auto & group : order) {
    auto * form = new QFormLayout;
    for (const auto & name : groups[group]) {
      const auto dot = name.find('.');
      const std::string label = dot == std::string::npos ? name : name.substr(dot + 1);
      form->addRow(QString::fromStdString(label), makeEditor(name, params_.at(name)));
    }
    vbox->addWidget(makeGroup(QString::fromStdString(group), form));
  }
  vbox->addStretch();
  scroll_->setWidget(content);  // deletes the previous form
  rendering_ = false;
}

QWidget * TuningPanel::makeEditor(const std::string & name, const rclcpp::Parameter & param)
{
  QWidget * editor = nullptr;
  switch (param.get_type()) {
    case ParamType::PARAMETER_BOOL: {
        auto * box = new QCheckBox;
        box->setChecked(param.as_bool());
        connect(box, &QCheckBox::toggled, this, [this, name](bool) {markDirty(name);});
        editor = box;
        break;
      }
    case ParamType::PARAMETER_INTEGER: {
        auto * spin = new QSpinBox;
        spin->setRange(-1000000000, 1000000000);
        spin->setValue(static_cast<int>(param.as_int()));
        spin->setKeyboardTracking(false);
        connect(
          spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [this, name](int) {markDirty(name);});
        editor = spin;
        break;
      }
    case ParamType::PARAMETER_DOUBLE: {
        auto * spin = new QDoubleSpinBox;
        spin->setRange(-1e6, 1e6);
        spin->setDecimals(4);
        spin->setValue(param.as_double());
        spin->setSingleStep(stepFor(param.as_double()));
        spin->setKeyboardTracking(false);
        connect(
          spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this, name](double) {markDirty(name);});
        editor = spin;
        break;
      }
    case ParamType::PARAMETER_STRING:
    case ParamType::PARAMETER_STRING_ARRAY: {
        auto * line = new QLineEdit;
        if (param.get_type() == ParamType::PARAMETER_STRING) {
          line->setText(QString::fromStdString(param.as_string()));
        } else {
          QStringList items;
          for (const auto & s : param.as_string_array()) {
            items << QString::fromStdString(s);
          }
          line->setText(items.join(", "));
        }
        connect(line, &QLineEdit::editingFinished, this, [this, name]() {markDirty(name);});
        editor = line;
        break;
      }
    default: {
        auto * line = new QLineEdit(QString::fromStdString(param.value_to_string()));
        line->setReadOnly(true);
        editor = line;
        break;
      }
  }
  editors_[name] = editor;
  return editor;
}

rclcpp::Parameter TuningPanel::editorValue(const std::string & name) const
{
  const auto & original = params_.at(name);
  const auto it = editors_.find(name);
  if (it == editors_.end()) {
    return original;
  }
  QWidget * w = it->second;
  switch (original.get_type()) {
    case ParamType::PARAMETER_BOOL:
      return rclcpp::Parameter(name, static_cast<QCheckBox *>(w)->isChecked());
    case ParamType::PARAMETER_INTEGER:
      return rclcpp::Parameter(name, static_cast<int64_t>(static_cast<QSpinBox *>(w)->value()));
    case ParamType::PARAMETER_DOUBLE:
      return rclcpp::Parameter(name, static_cast<QDoubleSpinBox *>(w)->value());
    case ParamType::PARAMETER_STRING:
      return rclcpp::Parameter(name, static_cast<QLineEdit *>(w)->text().toStdString());
    case ParamType::PARAMETER_STRING_ARRAY: {
        std::vector<std::string> items;
        for (const auto & s : static_cast<QLineEdit *>(w)->text().split(',')) {
          items.push_back(s.trimmed().toStdString());
        }
        return rclcpp::Parameter(name, items);
      }
    default:
      return original;
  }
}

void TuningPanel::markDirty(const std::string & name)
{
  if (rendering_) {
    return;
  }
  dirty_.insert(name);
  if (live_check_->isChecked()) {
    onApply();
  } else {
    setStatus(status_label_, std::to_string(dirty_.size()) + " change(s) pending", "#ffff88");
  }
}

void TuningPanel::onApply()
{
  if (!client_ || dirty_.empty()) {
    return;
  }
  std::vector<rclcpp::Parameter> changes;
  for (const auto & name : dirty_) {
    changes.push_back(editorValue(name));
  }
  sendParameters(changes, false);
}

void TuningPanel::onSelectorActivated(int index)
{
  const std::string selector = selectorName();
  if (rendering_ || !client_ || selector.empty() || index < 0) {
    return;
  }
  const std::string value = selector_combo_->itemText(index).toStdString();
  sendParameters({rclcpp::Parameter(selector, value)}, true);
}

void TuningPanel::sendParameters(const std::vector<rclcpp::Parameter> & params, bool rerender)
{
  client_->set_parameters(
    params,
    [this, params, rerender](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      std::string failures;
      for (size_t i = 0; i < results.size() && i < params.size(); ++i) {
        const std::string & name = params[i].get_name();
        if (results[i].successful) {
          params_.insert_or_assign(name, params[i]);
          dirty_.erase(name);
        } else {
          failures += name + ": " + results[i].reason + "\n";
        }
      }
      if (rerender) {
        render();
      }
      if (failures.empty()) {
        setStatus(
          status_label_, "applied " + std::to_string(params.size()) + " parameter(s)", "#88ff88");
      } else {
        setStatus(status_label_, "rejected:\n" + failures, "#ff8888");
      }
    });
}

void TuningPanel::onSaveYaml()
{
  if (params_.empty()) {
    setStatus(status_label_, "nothing loaded", "#ffcc00");
    return;
  }
  const std::string node_name = target_.substr(target_.find_last_of('/') + 1);
  const QString path = QFileDialog::getSaveFileName(
    this, "Save parameters", QDir::homePath() + "/" + QString::fromStdString(node_name) +
    "_tuned.yaml", "YAML (*.yaml)");
  if (path.isEmpty()) {
    return;
  }
  std::ofstream out(path.toStdString());
  out << node_name << ":\n  ros__parameters:\n";
  for (const auto & [name, param] : params_) {
    (void)param;
    out << "    " << name << ": " << yamlValue(editorValue(name)) << "\n";
  }
  setStatus(
    status_label_, out.good() ? "saved " + path.toStdString() : "could not write file",
    out.good() ? "#88ff88" : "#ff8888");
}

}  // namespace molly_rviz

PLUGINLIB_EXPORT_CLASS(molly_rviz::TuningPanel, rviz_common::Panel)
