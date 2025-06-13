#pragma once

#include "Eigen/Dense"
#include "units.h"

namespace vilot {

/**
 * @brief Madgwick AHRS (Attitude and Heading Reference System) filter
 * implementation
 *
 * This class implements the Madgwick AHRS algorithm for sensor fusion of
 * gyroscope and accelerometer data to estimate orientation in 3D space.
 * The filter uses a quaternion representation and gradient descent algorithm
 * to compute the orientation estimate.
 *
 * Coordinate System: x forward, y left, z up (NED - North East Down convention)
 *
 * The algorithm was originally developed by Sebastian O.H. Madgwick and is
 * particularly suited for real-time applications due to its computational
 * efficiency.
 *
 * @note This implementation does not include magnetometer fusion. For full AHRS
 *       functionality with yaw drift correction, magnetometer data should be
 * added.
 *
 * Example usage:
 * @code
 * using namespace units::literals;
 *
 * // Create filter with 100Hz sample rate and default beta (0.3)
 * Madgwick filter(100_Hz, 0.1f);
 *
 * // Tare the filter to set initial orientation
 * filter.tare();
 *
 * // Update filter with sensor data
 * filter.calculate(gx, gy, gz, ax, ay, az);
 *
 * // Get orientation angles
 * auto roll = filter.roll();
 * auto pitch = filter.pitch();
 * auto yaw = filter.yaw();
 * @endcode
 */
class Madgwick {
  using radian_t = units::angle::radian_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;
  using meters_per_second_squared_t =
      units::acceleration::meters_per_second_squared_t;
  using hertz_t = units::frequency::hertz_t;

public:
  /**
   * @brief Default constructor is deleted to enforce proper initialization
   */
  Madgwick() = delete;

  /**
   * @brief Construct a new Madgwick filter
   *
   * @param sample_freq The sampling frequency of the sensor data in Hz
   * @param beta The filter gain parameter (default: 0.3)
   *             Higher values provide faster convergence but more noise
   * sensitivity Lower values provide slower convergence but better noise
   * rejection Typical range: 0.01 to 1.0
   */
  Madgwick(const hertz_t sample_freq, const float beta = 0.3);

  /**
   * @brief Tare (zero/calibrate) the filter orientation
   *
   * Sets the current orientation as the reference point by applying offsets.
   * This effectively zeros out the current roll, pitch, and yaw readings.
   *
   * @param yaw Target yaw angle after taring (default: 0 radians)
   * @param pitch Target pitch angle after taring (default: 0 radians)
   * @param roll Target roll angle after taring (default: 0 radians)
   */
  void tare(const radian_t yaw = radian_t(0),
            const radian_t pitch = radian_t(0),
            const radian_t roll = radian_t(0)) noexcept;

  /**
   * @brief Get the current roll angle
   *
   * Roll represents rotation around the x-axis (forward axis).
   * Positive roll means rotating towards the right side.
   *
   * @return The roll angle in radians, offset-corrected
   */
  const radian_t roll() const noexcept;

  /**
   * @brief Get the current pitch angle
   *
   * Pitch represents rotation around the y-axis (left axis).
   * Positive pitch means nose up rotation.
   *
   * @return The pitch angle in radians, offset-corrected
   */
  const radian_t pitch() const noexcept;

  /**
   * @brief Get the current yaw angle
   *
   * Yaw represents rotation around the z-axis (up axis).
   * Positive yaw means rotating counterclockwise when viewed from above.
   *
   * @note Without magnetometer data, yaw will drift over time due to
   *       gyroscope integration errors.
   *
   * @return The yaw angle in radians, offset-corrected
   */
  const radian_t yaw() const noexcept;

  /**
   * @brief Update the filter with new sensor measurements
   *
   * This is the main filter update function that performs sensor fusion
   * using the Madgwick algorithm. Call this method at the rate specified
   * in the constructor's sample_freq parameter.
   *
   * @param gx Gyroscope x-axis (roll rate) in radians per second
   * @param gy Gyroscope y-axis (pitch rate) in radians per second
   * @param gz Gyroscope z-axis (yaw rate) in radians per second
   * @param ax Accelerometer x-axis in meters per second squared
   * @param ay Accelerometer y-axis in meters per second squared
   * @param az Accelerometer z-axis in meters per second squared
   */
  void calculate(const radians_per_second_t gx, const radians_per_second_t gy,
                 const radians_per_second_t gz,
                 const meters_per_second_squared_t ax,
                 const meters_per_second_squared_t ay,
                 const meters_per_second_squared_t az) noexcept;

  /**
   * @brief Functional operator for updating the filter
   *
   * Convenience operator that calls calculate() with the same parameters.
   * Allows the filter object to be used as a function.
   *
   * @param gx Gyroscope x-axis (roll rate) in radians per second
   * @param gy Gyroscope y-axis (pitch rate) in radians per second
   * @param gz Gyroscope z-axis (yaw rate) in radians per second
   * @param ax Accelerometer x-axis in meters per second squared
   * @param ay Accelerometer y-axis in meters per second squared
   * @param az Accelerometer z-axis in meters per second squared
   */
  void operator()(const radians_per_second_t gx, const radians_per_second_t gy,
                  const radians_per_second_t gz,
                  const meters_per_second_squared_t ax,
                  const meters_per_second_squared_t ay,
                  const meters_per_second_squared_t az) noexcept;

  /**
   * @brief Get the current orientation as a quaternion
   *
   * Returns the current orientation estimate as an Eigen quaternion.
   * The quaternion follows the convention: w + xi + yj + zk where
   * w is the scalar part and (x,y,z) is the vector part.
   *
   * @return Eigen::Quaternionf representing the current orientation
   */
  Eigen::Quaternionf quaternion() const noexcept;

private:
  const float sample_freq; ///< Sample frequency in Hz
  const float beta;        ///< Filter gain parameter

  // Quaternion components (w, x, y, z)
  float q1 = 1.0; ///< Quaternion w component (scalar part)
  float q2;       ///< Quaternion x component
  float q3;       ///< Quaternion y component
  float q4;       ///< Quaternion z component

  // Angle offsets for taring functionality
  radian_t roll_offset;  ///< Roll angle offset from taring
  radian_t pitch_offset; ///< Pitch angle offset from taring
  radian_t yaw_offset;   ///< Yaw angle offset from taring
};

/**
 * @brief Fast inverse square root implementation
 *
 * Uses the famous "Quake III" fast inverse square root algorithm for
 * performance in real-time applications. This function computes 1/sqrt(value)
 * approximately 4x faster than the standard library implementation.
 *
 * @param value The input value (must be positive)
 * @return Approximate value of 1/sqrt(value)
 *
 * @see https://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
const float inv_sqrt(const float value) noexcept;

/**
 * @brief Apply angle offset and wrap result to (-180°, 180°] range
 *
 * Adds an offset to an angle and ensures the result is wrapped to the
 * standard angular range of (-π, π] radians or (-180°, 180°] degrees.
 * This is used internally for the taring functionality.
 *
 * @param angle The base angle in radians
 * @param offset The offset to add in radians
 * @return The offset angle wrapped to (-π, π] range
 */
units::angle::radian_t offset_angle(units::angle::radian_t angle,
                                    units::angle::radian_t offset) noexcept;

} // namespace vilot