#include "app/state_machine.h"

namespace bb {

StateMachine::StateMachine() : state_(AppState::SAFE_DISABLED) {}

AppState StateMachine::state() const { return state_; }

void StateMachine::reset() { state_ = AppState::SAFE_DISABLED; }

bool StateMachine::startSignCalibration() {
  if (state_ == AppState::RUNNING || state_ == AppState::FAULT) {
    return false;
  }
  state_ = AppState::CALIB_SIGN;
  return true;
}

bool StateMachine::finishSignCalibration(bool success) {
  if (state_ != AppState::CALIB_SIGN) {
    return false;
  }
  state_ = success ? AppState::READY : AppState::SAFE_DISABLED;
  return true;
}

bool StateMachine::requestRun(const AppConditions& cond) {
  if (state_ == AppState::FAULT) {
    return false;
  }
  if (!cond.sign_calibrated || !cond.zero_calibrated ||
      !cond.limits_calibrated || !cond.sensors_ok || cond.faults_active ||
      !cond.inner_loop_stable) {
    return false;
  }
  state_ = AppState::RUNNING;
  return true;
}

void StateMachine::requestStop() {
  if (state_ == AppState::RUNNING || state_ == AppState::CALIB_SCALE) {
    state_ = AppState::READY;
    return;
  }
  if (state_ == AppState::CALIB_SIGN) {
    state_ = AppState::SAFE_DISABLED;
  }
}

void StateMachine::requestFault() { state_ = AppState::FAULT; }

bool StateMachine::clearFault(const AppConditions& cond) {
  if (state_ != AppState::FAULT) {
    return false;
  }
  state_ = (cond.sign_calibrated && cond.zero_calibrated &&
            cond.limits_calibrated)
               ? AppState::READY
               : AppState::SAFE_DISABLED;
  return true;
}

}  // namespace bb
