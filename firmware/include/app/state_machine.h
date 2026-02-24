#pragma once

#include "types.h"

namespace bb {

struct AppConditions {
  bool sign_calibrated = false;
  bool zero_calibrated = false;
  bool limits_calibrated = false;
  bool sensors_ok = false;
  bool faults_active = false;
  bool inner_loop_stable = false;
};

class StateMachine {
 public:
  StateMachine();

  AppState state() const;
  void reset();

  bool startSignCalibration();
  bool finishSignCalibration(bool success);

  bool requestRun(const AppConditions& cond);
  void requestStop();

  void requestFault();
  bool clearFault(const AppConditions& cond);

 private:
  AppState state_;
};

}  // namespace bb
