#include "selfdrive/ui/sunnypilot/qt/offroad/settings/tuning_panel.h"

#include <QShowEvent>
#include "common/util.h"
#include "common/params.h"
#include "cereal/messaging/messaging.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/controls.h"

TuningPanel::TuningPanel(SettingsWindowSP *parent) : QFrame(parent) {
  QWidget *main_widget = new QWidget(this);
  QVBoxLayout *main_layout = new QVBoxLayout(main_widget);
  main_layout->setContentsMargins(50, 20, 50, 20);

  list = new ListWidget(this, false);

  // Kp Low Speed - for custom error calculation
  kp_low_speed_control = new OptionControlSP(
    "KpLowSpeed",
    tr("Kp Low Speed"),
    tr("Proportional gain multiplier at low speeds (6.7 m/s). Used in custom error calculation."),
    "",
    {50, 500},  // min: 0.50, max: 5.00 (scaled by 100)
    5,          // step: 0.05 (scaled by 100)
    false,
    nullptr,
    true,
    false
  );
  // Set default value if not already set
  if (params.get("KpLowSpeed").empty()) {
    params.put("KpLowSpeed", "1.0");
  }
  QObject::connect(kp_low_speed_control, &OptionControlSP::updateLabels, this, &TuningPanel::refreshLabels);
  list->addItem(kp_low_speed_control);

  list->addItem(vertical_space());
  list->addItem(horizontal_line());

  // Kp High Speed - for custom error calculation
  kp_high_speed_control = new OptionControlSP(
    "KpHighSpeed",
    tr("Kp High Speed"),
    tr("Proportional gain multiplier at high speeds (33.5 m/s). Used in custom error calculation."),
    "",
    {50, 500},  // min: 0.50, max: 5.00 (scaled by 100)
    5,          // step: 0.05 (scaled by 100)
    false,
    nullptr,
    true,
    false
  );
  // Set default value if not already set
  if (params.get("KpHighSpeed").empty()) {
    params.put("KpHighSpeed", "1.0");
  }
  QObject::connect(kp_high_speed_control, &OptionControlSP::updateLabels, this, &TuningPanel::refreshLabels);
  list->addItem(kp_high_speed_control);

  main_layout->addWidget(list);
  main_layout->addStretch();

  scroller = new ScrollViewSP(main_widget, this);
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(scroller);

  // Initial label update
  refreshLabels();
}

void TuningPanel::refreshLabels() {
  // Update label text for each control
  if (kp_low_speed_control) {
    QString kp_low_speed_str = QString::fromStdString(params.get("KpLowSpeed"));
    double kp_low_speed_val = kp_low_speed_str.isEmpty() ? 1.0 : kp_low_speed_str.toDouble();
    kp_low_speed_control->setLabel(QString::number(kp_low_speed_val, 'f', 2));
  }
  if (kp_high_speed_control) {
    QString kp_high_speed_str = QString::fromStdString(params.get("KpHighSpeed"));
    double kp_high_speed_val = kp_high_speed_str.isEmpty() ? 1.0 : kp_high_speed_str.toDouble();
    kp_high_speed_control->setLabel(QString::number(kp_high_speed_val, 'f', 2));
  }
}

void TuningPanel::showEvent(QShowEvent *event) {
  QFrame::showEvent(event);
  // Refresh labels when panel is shown
  refreshLabels();
}
