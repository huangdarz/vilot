#include "vilot/pid.h"
#include <algorithm>
#include <cmath>

using namespace units::time;

namespace vilot {

PidController::PidController(PidConstants constants, const second_t sample_time)
    : kP(constants.kP), kI(constants.kI), kD(constants.kD), kF(constants.kF),
      tau(0.01), sample_time(sample_time), abs_max_output(600.0),
      integrator(0.0), prev_error(0.0), differentiator(0.0),
      prev_measurement(0.0), is_continuous_input(false),
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

float PidController::get_abs_max_output() { return this->abs_max_output; }

float PidController::get_min_input() {
  return this->input_modulus.get_min_input();
}

float PidController::get_max_input() {
  return this->input_modulus.get_max_input();
}

bool PidController::get_continuous_input() { return this->is_continuous_input; }

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

NonlinearPidController::NonlinearPidController(
    const NonLinearPidConstants constants,
    const units::time::second_t sample_time)
    : kPMax(constants.kPMax), kPMin(constants.kPMin), alphaP(constants.alphaP),
      kIMax(constants.kIMax), kIMin(constants.kIMin), alphaI(constants.alphaI),
      kD(constants.kD), kF(constants.kF), tau(0.01), sample_time(sample_time),
      abs_max_output(600.0), integrator(0.0), prev_error(0.0),
      differentiator(0.0), prev_measurement(0.0), is_continuous_input(false),
      input_modulus(-M_PI, M_PI) {}

float NonlinearPidController::calculate(const float measurement,
                                        const float setpoint) {
  float error = setpoint - measurement;
  error = this->is_continuous_input ? this->input_modulus.apply(error) : error;

  // proportional
  float kP = kPMax - (kPMax - kPMin) * (1.0 + alphaP * std::abs(error)) *
                         std::exp(-alphaP * std::abs(error));
  float p_output = kP * error;

  // integral
  if (this->prev_error * error < 0) {
    this->integrator = 0;
  }

  float kI = kIMax - (kIMax - kIMin) * (1.0 + alphaI * std::abs(error)) *
                         std::exp(-alphaI * std::abs(error));
  kI = 1.0 / kI;

  this->integrator +=
      0.5 * kI * this->sample_time() * (error + this->prev_error);

  // dynamic integrator clamping for anti-windup
  float max_integral =
      this->abs_max_output > p_output ? this->abs_max_output - p_output : 0.0;
  float min_integral =
      -this->abs_max_output < p_output ? -this->abs_max_output - p_output : 0.0;

  this->integrator = std::clamp(this->integrator, min_integral, max_integral);

  // differential
  // derivative on measurement
  this->differentiator =
      -(2.0 * this->kD * (measurement - this->prev_measurement) +
        (2.0 * this->tau - this->sample_time()) * this->differentiator) /
      (2.0 * this->tau + this->sample_time());

  // feedforward
  float f_output = this->kF * setpoint;

  // output
  float output = p_output + this->integrator + this->differentiator + f_output;
  output = std::clamp(output, -abs_max_output, abs_max_output);

  // update previous
  this->prev_error = error;
  this->prev_measurement = measurement;

  return output;
}

void NonlinearPidController::reset() {
  this->integrator = 0.0;
  this->prev_error = 0.0;
  this->differentiator = 0.0;
  this->prev_measurement = 0.0;
}

float NonlinearPidController::get_abs_max_output() {
  return this->abs_max_output;
}

float NonlinearPidController::get_min_input() {
  return this->input_modulus.get_min_input();
}

float NonlinearPidController::get_max_input() {
  return this->input_modulus.get_max_input();
}

bool NonlinearPidController::get_continuous_input() {
  return this->is_continuous_input;
}

void NonlinearPidController::set_abs_max_output(float abs_max_output) {
  this->abs_max_output = abs_max_output;
}

void NonlinearPidController::set_min_input(float min_input) {
  this->input_modulus =
      InputModulusFilter(min_input, this->input_modulus.get_max_input());
}

void NonlinearPidController::set_max_input(float max_input) {
  this->input_modulus =
      InputModulusFilter(this->input_modulus.get_min_input(), max_input);
}

void NonlinearPidController::set_continuous_input(bool is_continuous_input) {
  this->is_continuous_input = is_continuous_input;
}

} // namespace vilot
