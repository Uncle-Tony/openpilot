#include "selfdrive/ui/sunnypilot/qt/offroad/settings/tuning_panel.h"

#include <QShowEvent>
#include <QJsonDocument>
#include <QJsonObject>
#include "common/util.h"
#include "common/params.h"
#include "cereal/messaging/messaging.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/controls.h"

TuningPanel::TuningPanel(SettingsWindowSP *parent) : QFrame(parent) {
  QWidget *main_widget = new QWidget(this);
  QVBoxLayout *main_layout = new QVBoxLayout(main_widget);
  main_layout->setContentsMargins(50, 20, 50, 20);

  list = new ListWidget(this, false);

  // Load JSON - show custom values if set, otherwise show defaults from car interface or code defaults
  QJsonObject tuning_data = loadJSONParam();
  bool has_custom_tuning = !params.get("LateralTuningParams").empty() && tuning_data.contains("kp");

  // Defaults from configure_torque_tune (code defaults)
  double default_kp = 1.0, default_ki = 0.3, default_deadzone = 0.0;

  // Try to get defaults from car interface if car is installed
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!has_custom_tuning && !cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::TORQUE) {
      auto torque = CP.getLateralTuning().getTorque();
      default_kp = torque.getKp();
      default_ki = torque.getKi();
      default_deadzone = torque.getSteeringAngleDeadzoneDeg();
    }
  }
  // If no car installed, use code defaults (so user can still see values)
  // Note: Friction is dynamically managed by live torque learning, not tunable here

  // Proportional Gain (kp) - use temp param, sync to/from JSON
  kp_control = new OptionControlSP(
    "LateralTuningKpTemp",
    tr("Proportional Gain (kp)"),
    tr("Controls how quickly the system responds to lane position errors. Higher = faster response."),
    "",
    {50, 300},  // min: 0.50, max: 3.00 (scaled by 100)
    5,          // step: 0.05 (scaled by 100)
    false,      // inline_layout
    nullptr,    // valueMap
    true,       // scale_float
    false       // advancedControl
  );
  // Always set a value (custom tuning, car defaults, or code defaults) so user can see it
  if (has_custom_tuning && tuning_data.contains("kp")) {
    params.put("LateralTuningKpTemp", QString::number(tuning_data["kp"].toDouble(), 'f', 2).toStdString());
  } else {
    params.put("LateralTuningKpTemp", QString::number(default_kp, 'f', 2).toStdString());
  }
  QObject::connect(kp_control, &OptionControlSP::updateLabels, this, &TuningPanel::refreshLabels);
  list->addItem(kp_control);

  list->addItem(vertical_space());
  list->addItem(horizontal_line());

  // Integral Gain (ki)
  ki_control = new OptionControlSP(
    "LateralTuningKiTemp",
    tr("Integral Gain (ki)"),
    tr("Controls steady-state tracking and eliminates persistent drift. Higher = better centering."),
    "",
    {10, 100},  // min: 0.10, max: 1.00 (scaled by 100)
    2,          // step: 0.02 (scaled by 100)
    false,
    nullptr,
    true,
    false
  );
  // Always set a value
  if (has_custom_tuning && tuning_data.contains("ki")) {
    params.put("LateralTuningKiTemp", QString::number(tuning_data["ki"].toDouble(), 'f', 2).toStdString());
  } else {
    params.put("LateralTuningKiTemp", QString::number(default_ki, 'f', 2).toStdString());
  }
  QObject::connect(ki_control, &OptionControlSP::updateLabels, this, &TuningPanel::refreshLabels);
  list->addItem(ki_control);

  list->addItem(vertical_space());
  list->addItem(horizontal_line());

  // Note: Friction is dynamically managed by SunnyPilot's live torque learning
  // and cannot be manually tuned via this menu

  // Steering Angle Deadzone
  deadzone_control = new OptionControlSP(
    "LateralTuningDeadzoneTemp",
    tr("Steering Angle Deadzone (deg)"),
    tr("Prevents overcorrection for very small errors. Higher = ignores smaller deviations."),
    "",
    {0, 50},    // min: 0.00, max: 0.50 (scaled by 100)
    1,          // step: 0.01 (scaled by 100)
    false,
    nullptr,
    true,
    false
  );
  // Always set a value
  if (has_custom_tuning && tuning_data.contains("deadzone")) {
    params.put("LateralTuningDeadzoneTemp", QString::number(tuning_data["deadzone"].toDouble(), 'f', 2).toStdString());
  } else {
    params.put("LateralTuningDeadzoneTemp", QString::number(default_deadzone, 'f', 2).toStdString());
  }
  QObject::connect(deadzone_control, &OptionControlSP::updateLabels, this, &TuningPanel::refreshLabels);
  list->addItem(deadzone_control);

  main_layout->addWidget(list);
  main_layout->addStretch();

  scroller = new ScrollViewSP(main_widget, this);
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(scroller);

  // Initial label update
  refreshLabels();
}

QJsonObject TuningPanel::loadJSONParam() {
  QString json_str = QString::fromStdString(params.get("LateralTuningParams"));
  if (json_str.isEmpty()) {
    // Return empty object if param doesn't exist (no custom tuning set)
    return QJsonObject();
  }

  QJsonParseError error;
  QJsonDocument doc = QJsonDocument::fromJson(json_str.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) {
    // Invalid JSON - return empty object
    return QJsonObject();
  }

  return doc.object();
}

void TuningPanel::refreshLabels() {
  // Update label text for each control and save to JSON
  if (kp_control) {
    double kp_val = QString::fromStdString(params.get("LateralTuningKpTemp")).toDouble();
    kp_control->setLabel(QString::number(kp_val, 'f', 2));
  }
  if (ki_control) {
    double ki_val = QString::fromStdString(params.get("LateralTuningKiTemp")).toDouble();
    ki_control->setLabel(QString::number(ki_val, 'f', 2));
  }
  if (deadzone_control) {
    double deadzone_val = QString::fromStdString(params.get("LateralTuningDeadzoneTemp")).toDouble();
    deadzone_control->setLabel(QString::number(deadzone_val, 'f', 2));
  }

  // Also update JSON param
  updateJSONParam();
}

void TuningPanel::updateJSONParam() {
  // Load current JSON to preserve any other fields
  QJsonObject tuning_data = loadJSONParam();

  // Get values from temp params (controls write to these)
  QString kp_str = QString::fromStdString(params.get("LateralTuningKpTemp"));
  QString ki_str = QString::fromStdString(params.get("LateralTuningKiTemp"));
  QString deadzone_str = QString::fromStdString(params.get("LateralTuningDeadzoneTemp"));

  // Only update if values are set (non-empty)
  if (!kp_str.isEmpty()) {
    tuning_data["kp"] = kp_str.toDouble();
  }
  if (!ki_str.isEmpty()) {
    tuning_data["ki"] = ki_str.toDouble();
  }
  if (!deadzone_str.isEmpty()) {
    tuning_data["deadzone"] = deadzone_str.toDouble();
  }
  // Note: friction removed - it's dynamically managed by live torque learning

  // Save back to JSON param (matches BluePilot approach)
  QJsonDocument doc(tuning_data);
  params.put("LateralTuningParams", doc.toJson(QJsonDocument::Compact).toStdString());
}

void TuningPanel::showEvent(QShowEvent *event) {
  QFrame::showEvent(event);
  // Reload JSON when panel is shown and sync to temp params
  // Only set values if custom tuning exists (don't show defaults)
  QJsonObject tuning_data = loadJSONParam();
  bool has_custom_tuning = !params.get("LateralTuningParams").empty() && !tuning_data.isEmpty();

  if (has_custom_tuning) {
    if (tuning_data.contains("kp")) {
      params.put("LateralTuningKpTemp", QString::number(tuning_data["kp"].toDouble(), 'f', 2).toStdString());
    }
    if (tuning_data.contains("ki")) {
      params.put("LateralTuningKiTemp", QString::number(tuning_data["ki"].toDouble(), 'f', 2).toStdString());
    }
    if (tuning_data.contains("deadzone")) {
      params.put("LateralTuningDeadzoneTemp", QString::number(tuning_data["deadzone"].toDouble(), 'f', 2).toStdString());
    }
  } else {
    // Clear temp params so controls show defaults
    params.remove("LateralTuningKpTemp");
    params.remove("LateralTuningKiTemp");
    params.remove("LateralTuningDeadzoneTemp");
  }
}

