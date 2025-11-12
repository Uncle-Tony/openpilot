#include "selfdrive/ui/sunnypilot/qt/offroad/settings/tuning_panel.h"

#include <QShowEvent>
#include <QJsonDocument>
#include <QJsonObject>
#include "common/util.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/controls.h"

TuningPanel::TuningPanel(SettingsWindowSP *parent) : QFrame(parent) {
  QWidget *main_widget = new QWidget(this);
  QVBoxLayout *main_layout = new QVBoxLayout(main_widget);
  main_layout->setContentsMargins(50, 20, 50, 20);

  list = new ListWidget(this, false);

  // Load or initialize JSON
  QJsonObject tuning_data = loadJSONParam();

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
  // Set initial value from JSON
  double kp_val = tuning_data.contains("kp") ? tuning_data["kp"].toDouble() : 1.50;
  params.put("LateralTuningKpTemp", QString::number(kp_val, 'f', 2).toStdString());
  QObject::connect(kp_control, &OptionControlSP::updateLabels, this, &TuningPanel::updateJSONParam);
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
  double ki_val = tuning_data.contains("ki") ? tuning_data["ki"].toDouble() : 0.40;
  params.put("LateralTuningKiTemp", QString::number(ki_val, 'f', 2).toStdString());
  QObject::connect(ki_control, &OptionControlSP::updateLabels, this, &TuningPanel::updateJSONParam);
  list->addItem(ki_control);

  list->addItem(vertical_space());
  list->addItem(horizontal_line());

  // Friction
  friction_control = new OptionControlSP(
    "LateralTuningFrictionTemp",
    tr("Friction"),
    tr("Provides resistance to lateral movement, reducing drift. Higher = less drift."),
    "",
    {5, 30},    // min: 0.05, max: 0.30 (scaled by 100)
    1,          // step: 0.01 (scaled by 100)
    false,
    nullptr,
    true,
    false
  );
  double friction_val = tuning_data.contains("friction") ? tuning_data["friction"].toDouble() : 0.12;
  params.put("LateralTuningFrictionTemp", QString::number(friction_val, 'f', 2).toStdString());
  QObject::connect(friction_control, &OptionControlSP::updateLabels, this, &TuningPanel::updateJSONParam);
  list->addItem(friction_control);

  list->addItem(vertical_space());
  list->addItem(horizontal_line());

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
  double deadzone_val = tuning_data.contains("deadzone") ? tuning_data["deadzone"].toDouble() : 0.10;
  params.put("LateralTuningDeadzoneTemp", QString::number(deadzone_val, 'f', 2).toStdString());
  QObject::connect(deadzone_control, &OptionControlSP::updateLabels, this, &TuningPanel::updateJSONParam);
  list->addItem(deadzone_control);

  main_layout->addWidget(list);
  main_layout->addStretch();

  scroller = new ScrollViewSP(main_widget, this);
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(scroller);
}

QJsonObject TuningPanel::loadJSONParam() {
  QString json_str = QString::fromStdString(params.get("LateralTuningParams"));
  if (json_str.isEmpty()) {
    // Return defaults if param doesn't exist
    QJsonObject defaults;
    defaults["kp"] = 1.50;
    defaults["ki"] = 0.40;
    defaults["friction"] = 0.12;
    defaults["deadzone"] = 0.10;
    return defaults;
  }

  QJsonParseError error;
  QJsonDocument doc = QJsonDocument::fromJson(json_str.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) {
    // Invalid JSON - return defaults
    QJsonObject defaults;
    defaults["kp"] = 1.50;
    defaults["ki"] = 0.40;
    defaults["friction"] = 0.12;
    defaults["deadzone"] = 0.10;
    return defaults;
  }

  return doc.object();
}

void TuningPanel::updateJSONParam() {
  // Load current JSON to preserve any other fields
  QJsonObject tuning_data = loadJSONParam();

  // Update values from temp params (controls write to these)
  tuning_data["kp"] = QString::fromStdString(params.get("LateralTuningKpTemp")).toDouble();
  tuning_data["ki"] = QString::fromStdString(params.get("LateralTuningKiTemp")).toDouble();
  tuning_data["friction"] = QString::fromStdString(params.get("LateralTuningFrictionTemp")).toDouble();
  tuning_data["deadzone"] = QString::fromStdString(params.get("LateralTuningDeadzoneTemp")).toDouble();

  // Save back to JSON param (matches BluePilot approach)
  QJsonDocument doc(tuning_data);
  params.put("LateralTuningParams", doc.toJson(QJsonDocument::Compact).toStdString());
}

void TuningPanel::showEvent(QShowEvent *event) {
  QFrame::showEvent(event);
  // Reload JSON when panel is shown and sync to temp params
  QJsonObject tuning_data = loadJSONParam();
  params.put("LateralTuningKpTemp", QString::number(tuning_data.value("kp").toDouble(1.50), 'f', 2).toStdString());
  params.put("LateralTuningKiTemp", QString::number(tuning_data.value("ki").toDouble(0.40), 'f', 2).toStdString());
  params.put("LateralTuningFrictionTemp", QString::number(tuning_data.value("friction").toDouble(0.12), 'f', 2).toStdString());
  params.put("LateralTuningDeadzoneTemp", QString::number(tuning_data.value("deadzone").toDouble(0.10), 'f', 2).toStdString());
}

