#pragma once

namespace bb {

struct PIDGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;

  // Integral state limits (error * seconds), not output limits.
  float i_min = -1e9f;
  float i_max = 1e9f;

  // Controller output limits.
  float out_min = -1e9f;
  float out_max = 1e9f;
};

class PID {
 public:
  PID();
  explicit PID(const PIDGains& gains);

  void setGains(const PIDGains& gains);
  const PIDGains& gains() const;

  void reset();
  float update(float error, float dt_s);

  float integral() const;
  float lastError() const;

 private:
  PIDGains gains_;
  float integral_;
  float prev_error_;
  bool has_prev_;
};

}  // namespace bb
