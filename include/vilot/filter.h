#pragma once

#include "vilot/units.h"

namespace vilot {

/**
 * @brief Apply S-curve (exponential) filtering to input signal
 *
 * Applies an exponential S-curve transformation to the input signal, providing
 * smooth non-linear response characteristics. Lower values near zero are
 * dampened while higher values are preserved, creating a smooth transition.
 *
 * The S-curve filter is useful for:
 * - Joystick input smoothing
 * - Motor control with gentle acceleration curves
 * - Signal conditioning where fine control at low values is desired
 *
 * @param input The input value to be filtered
 * @param strength The strength of the S-curve effect (higher = more pronounced
 * curve)
 * @param max_input The maximum expected input value for normalization
 * @return The filtered output value
 *
 * @note This function is constexpr and can be evaluated at compile time
 */
constexpr float s_curve_filter(float input, float strength,
                               float max_input) noexcept;

/**
 * @brief Apply ramp curve filtering to input signal
 *
 * Applies a ramp curve transformation to the input signal, similar to S-curve
 * but with linear ramping characteristics. This provides smooth acceleration
 * and deceleration curves for control inputs.
 *
 * The ramp curve filter is useful for:
 * - Motor speed ramping
 * - Smooth transitions between control states
 * - Linear acceleration profiles
 *
 * @param input The input value to be filtered
 * @param strength The strength of the ramp effect
 * @param max_input The maximum expected input value for normalization
 * @return The filtered output value
 *
 * @note This function is constexpr and can be evaluated at compile time
 */
constexpr float ramp_curve_filter(float input, float strength,
                                  float max_input) noexcept;

/**
 * @brief Apply deadzone filtering to eliminate small unwanted signals
 *
 * Creates a deadzone around zero where small input values are set to zero,
 * while larger values are scaled proportionally. This eliminates noise,
 * drift, and unwanted small movements in control systems.
 *
 * The deadzone filter:
 * - Returns 0 for inputs within [-min_output, +min_output]
 * - Scales larger inputs proportionally to maintain smooth response
 * - Useful for joysticks, sensor noise elimination, and drift compensation
 *
 * @param input The input value to be filtered
 * @param min_output The minimum threshold below which output is zero
 * @param max_output The maximum expected output value for scaling
 * @return The filtered output value (0 in deadzone, scaled otherwise)
 *
 * @note This function is constexpr and can be evaluated at compile time
 */
constexpr float deadzone_filter(float input, float min_output,
                                float max_output) noexcept;

/**
 * @brief Biquadratic low-pass filter for signal smoothing
 *
 * Implements a second-order (biquad) low-pass filter using the Direct Form I
 * structure. This filter provides excellent noise reduction and signal
 * smoothing with configurable cutoff frequency and good stability
 * characteristics.
 *
 * Key features:
 * - Second-order filter with 12dB/octave rolloff
 * - Butterworth response (maximally flat passband)
 * - Excellent for sensor data smoothing
 * - Real-time capable with minimal computational overhead
 *
 * Typical applications:
 * - Sensor data filtering (IMU, encoders, analog sensors)
 * - Motor control signal conditioning
 * - Audio signal processing
 * - Noise reduction in feedback control systems
 *
 * @note The filter maintains internal state and is not thread-safe.
 *       Use separate instances for concurrent processing.
 */
class BiquadLowPassFilter {
public:
  BiquadLowPassFilter() = delete;

  /**
   * @brief Construct a biquad low-pass filter
   *
   * Initializes the filter with specified sample rate and cutoff frequency.
   * The filter coefficients are calculated using Butterworth design for
   * maximally flat response in the passband.
   *
   * @param sample_rate The sampling rate of the input signal in Hz
   * @param cutoff_freq The cutoff frequency in Hz (-3dB point)
   *
   * @note The cutoff frequency should be less than half the sample rate
   *       (Nyquist frequency) for proper operation
   */
  BiquadLowPassFilter(float sample_rate, float cutoff_freq);

  /**
   * @brief Reset the filter's internal state
   *
   * Clears the filter's delay elements (z1, z2), effectively restarting
   * the filter with no memory of previous inputs. Useful when switching
   * between different signal sources or restarting filtering operations.
   */
  void reset();

  /**
   * @brief Change the filter's cutoff frequency
   *
   * Recalculates and updates the filter coefficients for a new cutoff frequency
   * while preserving the current internal state. This allows dynamic adjustment
   * of filtering characteristics during operation.
   *
   * @param sample_rate The sampling rate of the input signal in Hz
   * @param cutoff_freq The new cutoff frequency in Hz (-3dB point)
   */
  void set_cutoff_frequency(float sample_rate, float cutoff_freq);

  /**
   * @brief Apply the filter to a single input sample
   *
   * Processes one input sample through the biquad filter and returns the
   * filtered output. This function maintains internal state and should be
   * called sequentially with input samples at the specified sample rate.
   *
   * @param input The input sample to be filtered
   * @return The filtered output sample
   *
   * @note For best performance, call this function at a consistent rate
   *       matching the sample_rate specified in the constructor
   */
  float apply(float input);

private:
  float a0, a1, a2, b1, b2; ///< Filter coefficients (feedforward and feedback)
  float z1, z2;             ///< Delay elements (previous input samples)

  /**
   * @brief Initialize filter coefficients
   *
   * Calculates the biquad filter coefficients using the bilinear transform
   * method for Butterworth low-pass filter design. This is called internally
   * by the constructor and set_cutoff_frequency().
   *
   * @param sample_rate The sampling rate in Hz
   * @param cutoff_freq The cutoff frequency in Hz
   */
  void init(float sample_rate, float cutoff_freq);
};

/**
 * @brief Apply exponential decay filtering (standalone function)
 *
 * Applies exponential decay filtering to blend two values over time.
 * This function implements the mathematical formula:
 * result = b + (a - b) * exp(-decay * dt)
 *
 * The filter provides smooth transitions between values 'a' and 'b' with
 * the rate controlled by the decay parameter and time step.
 *
 * @param a The starting value
 * @param b The target value
 * @param decay The decay rate constant (higher = faster transition)
 * @param dt The time step since last update
 * @return The filtered value between a and b
 *
 * @note This function is stateless and constexpr. For stateful filtering,
 *       use the ExpDecayFilter class.
 */
constexpr float exp_decay_filter(const float a, const float b,
                                 const float decay,
                                 const units::time::second_t dt) noexcept;

/**
 * @brief Exponential decay filter class for smooth signal transitions
 *
 * Implements a first-order exponential decay filter that smoothly transitions
 * between input values over time. This filter is excellent for:
 *
 * - Smooth motor speed transitions
 * - Sensor data smoothing with time-based decay
 * - Implementing inertia in control systems
 * - Creating natural-feeling UI animations
 *
 * The filter equation: output = input + (previous - input) * exp(-decay * dt)
 *
 * Characteristics:
 * - Higher decay values = faster response (less smoothing)
 * - Lower decay values = slower response (more smoothing)
 * - Time-aware filtering adapts to varying update rates
 * - Maintains one previous value for stateful operation
 *
 * @note This filter is not thread-safe. Use separate instances for concurrent
 * use.
 */
class ExpDecayFilter {
public:
  ExpDecayFilter() = delete;

  /**
   * @brief Construct an exponential decay filter
   *
   * @param decay The decay rate constant (higher values = faster response)
   *              Typical range: 0.1 (very smooth) to 10.0 (very responsive)
   */
  ExpDecayFilter(float decay);

  /**
   * @brief Apply the filter to an input value
   *
   * Processes the input through the exponential decay filter, smoothly
   * transitioning from the previous output towards the new input value.
   * The rate of transition depends on the decay parameter and time step.
   *
   * @param input The new input value
   * @param dt The time elapsed since the last update
   * @return The filtered output value
   *
   * @note The first call initializes the filter state with the input value
   */
  float apply(float input, units::time::second_t dt);

  /**
   * @brief Reset the filter state
   *
   * Resets the internal previous value to zero, causing the next apply()
   * call to start fresh. Useful when switching between different signal
   * sources or restarting filtering operations.
   */
  void reset();

private:
  float prev;  ///< Previous output value for state continuity
  float decay; ///< Decay rate constant controlling filter response speed
};

/**
 * @brief Apply input modulus filtering (standalone function)
 *
 * Wraps input values to stay within the specified range using modulus
 * arithmetic. This is useful for circular/angular values that need to
 * wrap around (like angles, rotational positions, or cyclic references).
 *
 * The function ensures the output is always within [min_input, max_input)
 * by wrapping values that exceed the bounds.
 *
 * Common applications:
 * - Angle normalization (e.g., keeping angles in [0, 360) or [-180, 180))
 * - Circular buffer indexing
 * - Periodic signal processing
 * - Rotational position control
 *
 * @param input The input value to be wrapped
 * @param min_input The minimum value of the range
 * @param max_input The maximum value of the range
 * @return The wrapped value within [min_input, max_input)
 *
 * @note For angular applications, consider using units::angle functions
 *       for more robust angle handling
 */
float input_modulus_filter(float input, float min_input, float max_input);

/**
 * @brief Input modulus filter class for continuous range wrapping
 *
 * Provides stateful modulus filtering for inputs that need to be constrained
 * to a specific range with wrap-around behavior. This is particularly useful
 * for handling circular or periodic quantities.
 *
 * Key features:
 * - Wraps input values to stay within [min_input, max_input)
 * - Handles both positive and negative overflow gracefully
 * - Maintains consistent range boundaries
 * - Zero computational overhead for in-range values
 *
 * Common use cases:
 * - Servo position control with limited rotation
 * - Compass heading normalization
 * - Circular buffer management
 * - Periodic waveform generation
 * - Game world coordinate wrapping
 *
 * @note This filter is stateless per-call but maintains configuration state.
 *       It is thread-safe for read operations but not for configuration
 * changes.
 */
class InputModulusFilter {
public:
  /**
   * @brief Construct an input modulus filter
   *
   * @param min_input The minimum value of the allowed range
   * @param max_input The maximum value of the allowed range
   *
   * @note min_input should be less than max_input for proper operation
   */
  InputModulusFilter(float min_input, float max_input);

  /**
   * @brief Apply modulus filtering to an input value
   *
   * Wraps the input value to ensure it falls within the configured range
   * [min_input, max_input). Values outside this range are wrapped around
   * using modulus arithmetic.
   *
   * @param input The input value to be wrapped
   * @return The wrapped value within the configured range
   */
  float apply(float input);

  /**
   * @brief Get the minimum input value of the range
   *
   * @return The minimum value of the configured range
   */
  float get_min_input();

  /**
   * @brief Get the maximum input value of the range
   *
   * @return The maximum value of the configured range
   */
  float get_max_input();

private:
  float min_input; ///< Minimum value of the allowed range
  float max_input; ///< Maximum value of the allowed range
};

} // namespace vilot
