#pragma once

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/settings.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"

class OptionControlSP; // Forward declaration

class TuningPanel : public QFrame {
  Q_OBJECT

public:
  explicit TuningPanel(SettingsWindowSP *parent = nullptr);
  void showEvent(QShowEvent *event) override;

private slots:
  void refreshLabels();

private:

  Params params;
  ScrollViewSP *scroller = nullptr;
  ListWidget *list = nullptr;
  OptionControlSP *kp_control = nullptr;
  OptionControlSP *ki_control = nullptr;
  OptionControlSP *deadzone_control = nullptr;
  OptionControlSP *kf_control = nullptr;
  OptionControlSP *latAccelFactor_control = nullptr;
  OptionControlSP *latAccelOffset_control = nullptr;
};

