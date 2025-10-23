#pragma once

#include "units.h"
#include "vilot/filter.h"
#include <functional>

namespace vilot {

/**
 * @brief Constants for PID controller configuration
 *
 * This structure holds the proportional, integral, derivative, and feedforward
 * gains used by the PID controller.
 */
struct PidConstants {
  float kP; ///< Proportional gain - affects how aggressively the controller
            ///< responds to current error
  float kI; ///< Integral gain - affects how aggressively the controller
            ///< responds to accumulated error over time
  float kD; ///< Derivative gain - affects how aggressively the controller
            ///< responds to rate of error change
  float kF =
      0.0; ///< Feedforward gain - provides predictive control based on setpoint
};

/**
 * @brief A standard PID controller implementation
 *
 * This class implements a discrete-time PID (Proportional-Integral-Derivative)
 * controller with optional feedforward control. The controller calculates a
 * control output based on the error between a setpoint and measured value.
 *
 * The PID algorithm implemented is:
 * output = kP * error + kI * integral_of_error + kD * derivative_of_error + kF
 * * setpoint
 *
 * Features:
 * - Configurable sample time
 * - Output saturation limiting
 * - Continuous input support (for circular values like angles)
 * - Integral windup prevention
 * - Derivative kick prevention through measurement-based differentiation
 *
 * @note The derivative term is calculated based on the measurement rather than
 * error to prevent derivative kick when the setpoint changes suddenly.
 */
class PidController {
public:
  PidController() = delete;

  /**
   * @brief Construct a new PID Controller
   *
   * @param constants PID gains (kP, kI, kD, kF)
   * @param sample_time Time between controller updates (default: 10ms)
   */
  PidController(
      PidConstants constants,
      const units::time::second_t sample_time = units::time::millisecond_t(10));

  /**
   * @brief Calculate the controller output
   *
   * Computes the PID control output based on the current measurement and
   * desired setpoint. This method should be called at regular intervals
   * matching the sample_time.
   *
   * @param measurement Current process variable value
   * @param setpoint Desired target value
   * @return Control output value (may be limited by abs_max_output)
   */
  float calculate(float measurement, float setpoint);

  /**
   * @brief Reset the controller state
   *
   * Clears the integral accumulator, derivative history, and other internal
   * state. Use this when starting control of a new process or after a
   * significant disturbance.
   */
  void reset();

  /**
   * @brief Get the absolute maximum output limit
   * @return Current absolute maximum output value
   */
  float get_abs_max_output();

  /**
   * @brief Get the minimum input value for continuous input mode
   * @return Minimum input value
   */
  float get_min_input();

  /**
   * @brief Get the maximum input value for continuous input mode
   * @return Maximum input value
   */
  float get_max_input();

  /**
   * @brief Check if continuous input mode is enabled
   * @return true if continuous input is enabled, false otherwise
   */
  bool get_continuous_input();

  /**
   * @brief Set the absolute maximum output limit
   *
   * The controller output will be clamped to [-abs_max_output,
   * +abs_max_output]. This helps prevent actuator saturation and integrator
   * windup.
   *
   * @param abs_max_output Maximum absolute output value (must be positive)
   */
  void set_abs_max_output(float abs_max_output);

  /**
   * @brief Set the minimum input value for continuous input mode
   *
   * Used when the input wraps around (e.g., angles from -180 to +180 degrees).
   *
   * @param min_input Minimum input value
   */
  void set_min_input(float min_input);

  /**
   * @brief Set the maximum input value for continuous input mode
   *
   * Used when the input wraps around (e.g., angles from -180 to +180 degrees).
   *
   * @param max_input Maximum input value
   */
  void set_max_input(float max_input);

  /**
   * @brief Enable or disable continuous input mode
   *
   * When enabled, the controller handles wraparound inputs (like angles)
   * correctly by calculating the shortest path between setpoint and
   * measurement.
   *
   * @param is_continuous_input true to enable continuous input, false to
   * disable
   */
  void set_continuous_input(bool is_continuous_input);

private:
  float kP;
  float kI;
  float kD;
  float kF;
  float tau;
  units::time::second_t sample_time;

  float abs_max_output;
  float integrator;
  float prev_error;
  float differentiator;
  float prev_measurement;
  bool is_continuous_input;

  InputModulusFilter input_modulus;
};

struct SettleCondition {
  // basically if its within the tolerance for a set amount of time
  std::function<bool(float, float)> condition;
  const uint32_t settle_time;
  uint32_t prev_time;
  bool prev_success;

  bool check(float measurement, float goal);
};

} // namespace vilot
