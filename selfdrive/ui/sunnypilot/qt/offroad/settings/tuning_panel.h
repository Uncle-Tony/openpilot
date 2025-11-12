#pragma once

#include <QJsonObject>
#include "selfdrive/ui/sunnypilot/qt/offroad/settings/settings.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"

class OptionControlSP; // Forward declaration

class TuningPanel : public QFrame {
  Q_OBJECT

public:
  explicit TuningPanel(SettingsWindowSP *parent = nullptr);
  void showEvent(QShowEvent *event) override;

private slots:
  void updateJSONParam();
  void refreshLabels();

private:
  QJsonObject loadJSONParam();

  Params params;
  ScrollViewSP *scroller = nullptr;
  ListWidget *list = nullptr;
  OptionControlSP *kp_control = nullptr;
  OptionControlSP *ki_control = nullptr;
  OptionControlSP *friction_control = nullptr;
  OptionControlSP *deadzone_control = nullptr;
};

