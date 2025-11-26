#include "vilot/pid.hpp"
#include <algorithm>
#include <cmath>
#include "pros/rtos.hpp"

using namespace units::time;

namespace vilot {

PidController::PidController(PidConstants constants, const second_t sample_time)
    : kP(constants.kP),
      kI(constants.kI),
      kD(constants.kD),
      kF(constants.kF),
      tau(0.01),
      sample_time(sample_time),
      abs_max_output(600.0),
      integrator(0.0),
      prev_error(0.0),
      differentiator(0.0),
      prev_measurement(0.0),
      is_continuous_input(false),
      input_modulus(-M_PI, M_PI) {}

float PidController::calculate(float measurement, float setpoint) {
  float error = setpoint - measurement;
  error = this->is_continuous_input ? this->input_modulus.apply(error) : error;

  // proportional
  float p_output = this->kP * error;

  // integral
  if (this->prev_error * error < 0) {
    this->integrator = 0;
  }

  this->integrator +=
      0.5 * this->kI * this->sample_time.value() * (error + this->prev_error);

  // dynamic integrator clamping for anti-windup
  float max_integral =
      this->abs_max_output > p_output ? this->abs_max_output - p_output : 0.0;
  float min_integral =
      -this->abs_max_output < p_output ? -this->abs_max_output - p_output : 0.0;

  this->integrator = std::clamp(this->integrator, min_integral, max_integral);

  // differential
  // derivative on measurement with low-pass filter
  float derivative_raw =
      -(measurement - this->prev_measurement) / this->sample_time.value();

  this->differentiator =
      (2.0 * this->tau * derivative_raw +
       (2.0 * this->tau - this->sample_time.value()) * this->differentiator) /
      (2.0 * this->tau + this->sample_time.value());

  float d_output = this->kD * this->differentiator;

  // feedforward
  float f_output = this->kF * setpoint;

  // output
  float output = p_output + this->integrator + d_output + f_output;
  output = std::clamp(output, -abs_max_output, abs_max_output);

  // update previous
  this->prev_error = error;
  this->prev_measurement = measurement;

  return output;
}

void PidController::reset() {
  this->integrator = 0.0;
  this->prev_error = 0.0;
  this->differentiator = 0.0;
  this->prev_measurement = 0.0;
}

float PidController::get_abs_max_output() {
  return this->abs_max_output;
}

float PidController::get_min_input() {
  return this->input_modulus.get_min_input();
}

float PidController::get_max_input() {
  return this->input_modulus.get_max_input();
}

bool PidController::get_continuous_input() {
  return this->is_continuous_input;
}

void PidController::set_abs_max_output(float abs_max_output) {
  this->abs_max_output = abs_max_output;
}

void PidController::set_min_input(float min_input) {
  this->input_modulus =
      InputModulusFilter(min_input, this->input_modulus.get_max_input());
}

void PidController::set_max_input(float max_input) {
  this->input_modulus =
      InputModulusFilter(this->input_modulus.get_min_input(), max_input);
}

void PidController::set_continuous_input(bool is_continuous_input) {
  this->is_continuous_input = is_continuous_input;
}

bool SettleCondition::check(float measurement, float goal) {
  if (this->condition(measurement, goal)) {
    if (!this->prev_success) {
      this->prev_time = pros::millis();
    }
    this->prev_success = true;
  } else {
    this->prev_success = false;
    this->prev_time = pros::millis();
  }
  return pros::millis() - this->prev_time >= this->settle_time_ms &&
         this->prev_success;
}

std::function<bool(float, float)> absolute_tolerance(float tolerance) noexcept {
  return [tolerance](const float m, const float g) {
    return std::fabs(m - g) < tolerance;
  };
}

}  // namespace vilot
