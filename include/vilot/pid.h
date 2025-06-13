#pragma once

#include "vilot/filter.h"
#include "vilot/units.h"

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

/**
 * @brief Constants for nonlinear PID controller configuration
 *
 * This structure holds the parameters for a nonlinear PID controller where
 * the proportional and integral gains vary based on the error magnitude.
 * Larger errors result in higher gains for more aggressive control response.
 */
struct NonLinearPidConstants {
  float kPMax;  ///< Maximum proportional gain (used for large errors)
  float kPMin;  ///< Minimum proportional gain (used for small errors)
  float alphaP; ///< Proportional gain transition parameter (affects gain curve
                ///< steepness)
  float kIMax;  ///< Maximum integral gain (used for large errors)
  float kIMin;  ///< Minimum integral gain (used for small errors)
  float alphaI; ///< Integral gain transition parameter (affects gain curve
                ///< steepness)
  float kD;     ///< Derivative gain (constant, not error-dependent)
  float kF = 0.0; ///< Feedforward gain
};

/**
 * @brief A nonlinear PID controller implementation
 *
 * This class implements a nonlinear PID controller where the proportional and
 * integral gains are dynamically adjusted based on the magnitude of the error.
 * This approach provides more aggressive control for large errors while
 * maintaining stability for small errors near the setpoint.
 *
 * The gain scheduling follows the paper:
 * "Nonlinear PID controller based on Lyapunov's stability theory for robot
 * manipulator"
 * https://www.sciencedirect.com/science/article/pii/S1474667017564526
 *
 * The proportional and integral gains are calculated as:
 * kP(e) = kPMin + (kPMax - kPMin) * (1 - exp(-alphaP * |e|))
 * kI(e) = kIMin + (kIMax - kIMin) * (1 - exp(-alphaI * |e|))
 *
 * Features:
 * - Error-dependent proportional and integral gains
 * - Improved transient response for large errors
 * - Better steady-state performance for small errors
 * - All standard PID features (output limiting, continuous input support, etc.)
 *
 * @note The derivative gain remains constant as it's typically less sensitive
 * to error magnitude and varying it can introduce instability.
 */
// https://www.sciencedirect.com/science/article/pii/S1474667017564526
class NonlinearPidController {
public:
  NonlinearPidController() = delete;

  /**
   * @brief Construct a new Nonlinear PID Controller with units-based sample
   * time
   *
   * @param constants Nonlinear PID gains and parameters
   * @param sample_time Time between controller updates (default: 10ms)
   */
  NonlinearPidController(
      const NonLinearPidConstants constants,
      const units::time::second_t sample_time = units::time::millisecond_t(10));

  /**
   * @brief Construct a new Nonlinear PID Controller with raw float sample time
   *
   * @param constants Nonlinear PID gains and parameters
   * @param sample_time Time between controller updates in seconds (default:
   * 0.01s)
   */
  NonlinearPidController(const NonLinearPidConstants constants,
                         const float sample_time = 0.01);

  /**
   * @brief Calculate the controller output using nonlinear gain scheduling
   *
   * Computes the nonlinear PID control output where proportional and integral
   * gains are adjusted based on the error magnitude. This provides more
   * aggressive control for large errors and finer control near the setpoint.
   *
   * @param measurement Current process variable value
   * @param setpoint Desired target value
   * @return Control output value (may be limited by abs_max_output)
   */
  float calculate(const float measurement, const float setpoint);

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
  // Nonlinear PID parameters
  float kPMax;  ///< Maximum proportional gain coefficient
  float kPMin;  ///< Minimum proportional gain coefficient
  float alphaP; ///< Exponential transition parameter for proportional gain
                ///< scheduling
  float kIMax;  ///< Maximum integral gain coefficient
  float kIMin;  ///< Minimum integral gain coefficient
  float
      alphaI; ///< Exponential transition parameter for integral gain scheduling
  float kD;   ///< Derivative gain coefficient (constant)
  float kF;   ///< Feedforward gain coefficient
  float tau;  ///< Low-pass filter time constant for derivative term
  units::time::second_t sample_time; ///< Controller update period

  // Controller limits and state
  float abs_max_output; ///< Maximum absolute value of controller output
  float integrator;     ///< Integral term accumulator (sum of errors over time)
  float prev_error;     ///< Previous error value for derivative calculation
  float differentiator; ///< Low-pass filtered derivative term
  float prev_measurement;   ///< Previous measurement value for derivative kick
                            ///< prevention
  bool is_continuous_input; ///< Flag indicating if input values wrap around
                            ///< (e.g., angles)

  InputModulusFilter
      input_modulus; ///< Filter for handling continuous/wraparound inputs
};

} // namespace vilot
