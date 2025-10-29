#pragma once

#include "Eigen/Dense"
#include "pros/imu.hpp"
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
  explicit Madgwick(hertz_t sample_freq, float beta = 0.3);

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
  void tare(radian_t yaw = radian_t(0), radian_t pitch = radian_t(0),
            radian_t roll = radian_t(0)) noexcept;

  /**
   * @brief Get the current roll angle
   *
   * Roll represents rotation around the x-axis (forward axis).
   * Positive roll means rotating towards the right side.
   *
   * @return The roll angle in radians, offset-corrected
   */
  [[nodiscard]] radian_t roll() const noexcept;

  /**
   * @brief Get the current pitch angle
   *
   * Pitch represents rotation around the y-axis (left axis).
   * Positive pitch means nose up rotation.
   *
   * @return The pitch angle in radians, offset-corrected
   */
  [[nodiscard]] radian_t pitch() const noexcept;

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
  [[nodiscard]] radian_t yaw() const noexcept;

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
  void calculate(radians_per_second_t gx, radians_per_second_t gy,
                 radians_per_second_t gz, meters_per_second_squared_t ax,
                 meters_per_second_squared_t ay,
                 meters_per_second_squared_t az) noexcept;

  /**
   * @brief Get the current orientation as a quaternion
   *
   * Returns the current orientation estimate as an Eigen quaternion.
   * The quaternion follows the convention: w + xi + yj + zk where
   * w is the scalar part and (x,y,z) is the vector part.
   *
   * @return Quaternion representing the current orientation
   */
  [[nodiscard]] Eigen::Quaternionf quaternion() const noexcept;

private:
  const float sample_freq; ///< Sample frequency in Hz
  const float beta;        ///< Filter gain parameter

  // Quaternion components (w, x, y, z)
  float q1 = 1; // 0.7071068; // 1 ///< Quaternion w component (scalar part)
  float q2 = 0; // 0.7071068; ///< Quaternion x component
  float q3 = 0; ///< Quaternion y component
  float q4 = 0; ///< Quaternion z component

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
float inv_sqrt(float value) noexcept;

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

namespace vilot::device {

/**
 * @brief High-level IMU (Inertial Measurement Unit) wrapper with sensor fusion
 *
 * This class provides a high-level interface to VEX V5 IMU sensors with
 * integrated Madgwick AHRS sensor fusion filtering. It wraps the PROS IMU API
 * and automatically applies sensor fusion to provide stable and accurate
 * orientation estimates.
 *
 * Key Features:
 * - Automatic sensor fusion using Madgwick AHRS algorithm
 * - Thread-safe operation with mutex protection
 * - Built-in calibration and drift correction
 * - Continuous background sensor reading at 100Hz
 * - Stable heading output with automatic zeroing
 *
 * Coordinate System: Same as PROS IMU (x forward, y left, z up)
 *
 * The class automatically:
 * 1. Starts a background task for continuous sensor reading
 * 2. Applies Madgwick filtering to raw sensor data
 * 3. Calibrates the sensor on startup
 * 4. Provides thread-safe access to filtered orientation data
 *
 * @note This class is designed for robotics applications where stable
 * orientation estimation is critical, such as autonomous navigation systems.
 *
 * Example usage:
 * @code
 * using namespace vilot::device;
 * using namespace units::literals;
 *
 * // Create IMU on port 1 with custom beta parameter
 * Imu imu(1, 0.1f);
 *
 * // Start calibration (blocks until stable)
 * imu.start();
 *
 * // Reset orientation to zero
 * imu.reset();
 *
 * // Get current heading
 * auto heading = imu.get_heading();
 *
 * // Get port number
 * uint8_t port = imu.get_port();
 * @endcode
 */
class Imu {
  using radian_t = units::angle::radian_t;
  using degree_t = units::angle::degree_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;
  using meters_per_second_squared_t =
      units::acceleration::meters_per_second_squared_t;
  using hertz_t = units::frequency::hertz_t;

public:
  /**
   * @brief Default constructor is deleted to enforce proper initialization
   */
  Imu() = delete;

  /**
   * @brief Copy constructor is deleted (non-copyable due to hardware resource)
   */
  Imu(const Imu &) = delete;

  /**
   * @brief Copy assignment operator is deleted (non-copyable due to hardware
   * resource)
   */
  Imu &operator=(const Imu &) = delete;

  /**
   * @brief Move constructor is deleted (non-moveable due to running task)
   */
  Imu(Imu &&) = delete;

  /**
   * @brief Move assignment operator is deleted (non-moveable due to running
   * task)
   */
  Imu &operator=(Imu &&) = delete;

  /**
   * @brief Construct a new IMU object
   *
   * Creates a new IMU wrapper that interfaces with a VEX V5 IMU sensor on the
   * specified port. The constructor automatically starts a background task that
   * continuously reads sensor data and applies Madgwick filtering.
   *
   * @param port The V5 port number (1-21) where the IMU sensor is connected
   * @param beta The Madgwick filter gain parameter (default: 0.3)
   *             Higher values provide faster convergence but more noise
   * sensitivity Lower values provide slower convergence but better noise
   * rejection Typical range: 0.01 to 1.0
   *
   * @note After construction, call start() to perform calibration before using
   *       the sensor for navigation or control purposes.
   *
   * @see Madgwick::Madgwick() for more details on the beta parameter
   */
  explicit Imu(uint8_t port, float beta = 0.3);

  /**
   * @brief Reset/tare the IMU orientation
   *
   * Sets the current orientation as the new reference point by applying offsets
   * to the filter. This effectively zeros out the current orientation readings
   * to the specified target values.
   *
   * @param yaw Target yaw angle after reset (default: 0 radians)
   * @param pitch Target pitch angle after reset (default: 0 radians)
   * @param roll Target roll angle after reset (default: 0 radians)
   *
   * @note This operation is thread-safe and can be called while the sensor
   *       is actively being read by the background task.
   */
  void reset(radian_t yaw = radian_t(0), radian_t pitch = radian_t(0),
             radian_t roll = radian_t(0));

  /**
   * @brief Start IMU calibration and operation
   *
   * This function performs the complete IMU initialization sequence:
   * 1. Resets the underlying PROS IMU sensor (blocking)
   * 2. Starts the background sensor reading task
   * 3. Waits for the sensor readings to stabilize (drift < 0.005%)
   * 4. Automatically tares the filter to zero orientation
   * 5. Marks calibration as complete
   *
   * This function blocks until the calibration process is complete, which
   * typically takes 2-5 seconds depending on environmental conditions.
   *
   * @note This function MUST be called before using the IMU for any control
   *       or navigation purposes. The sensor will not provide accurate readings
   *       until calibration is complete.
   *
   * @warning Do not move the robot during calibration as this will affect
   *          the accuracy of the orientation reference.
   */
  bool start();

  /**
   * @brief Get the current heading (yaw) angle
   *
   * Returns the current heading angle from the Madgwick filter, which
   * represents rotation around the z-axis (vertical axis). The heading is
   * relative to the orientation when the sensor was last tared/reset.
   *
   * @return The current heading angle in degrees
   *
   * @note This function is thread-safe and can be called from any task.
   *       The returned value is automatically offset-corrected based on the
   *       last tare/reset operation.
   */
  degree_t get_heading() const;

  degree_t get_yaw() const;
  degree_t get_pitch() const;
  degree_t get_roll() const;

  /**
   * @brief Get the port number of the IMU sensor
   *
   * Returns the V5 port number where this IMU sensor is connected.
   *
   * @return The port number (1-21) of the connected IMU sensor
   */
  uint8_t get_port() const;

private:
  /**
   * @brief Background sensor reading and filtering loop
   *
   * This function runs continuously in a separate task and performs:
   * 1. Reads raw gyroscope and accelerometer data from the PROS IMU
   * 2. Converts units to match the Madgwick filter requirements
   * 3. Updates the Madgwick filter with new sensor data
   * 4. Maintains a 100Hz update rate (10ms intervals)
   *
   * The function is thread-safe and uses mutex protection when updating
   * the filter state to prevent race conditions with getter functions.
   *
   * @note This function is automatically started when the IMU object is created
   *       and runs until the object is destroyed.
   */
  [[noreturn]] void update();

  pros::Task task;         ///< Background task for sensor reading
  pros::Imu inertial;      ///< PROS IMU sensor interface
  Madgwick filter;         ///< Madgwick AHRS filter for sensor fusion
  bool is_calibrating;     ///< Flag indicating calibration status
  mutable pros::Mutex mut; ///< Mutex for thread-safe filter access
};

} // namespace vilot::device
