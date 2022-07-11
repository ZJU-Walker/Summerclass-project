#include "SunPID.h"
PID::PID(float min_val, float max_val, float kp, float ki, float kd)
    : min_val_(min_val), max_val_(max_val), kp_(kp), ki_(ki), kd_(kd) {}

double PID::Compute(float setpoint, float measured_value) {
  double error;
  double pid;
  error = setpoint - measured_value;
  integral_ += error;

  derivative_ = error - prev_error_;
  if (setpoint == 0 && error == 0) {
    integral_ = 0;
  }
  //抗积分饱和——限幅方法
  pid = (kp_ * error) + constrain(ki_ * integral_, min_val_, max_val_) +
        (kd_ * derivative_);
  prev_error_ = error;
  //抗执行器饱和——限幅方法
  return constrain(pid, min_val_, max_val_);
}

void PID::UpdateTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
