#include "vilot/inertial.hpp"
#include "etl/circular_buffer.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstdio>

namespace vilot {

using namespace units::literals;
using namespace units::angle;
using namespace units::acceleration;
using namespace units::angular_velocity;

Madgwick::Madgwick(const hertz_t sample_freq, const float beta)
    : sample_freq(sample_freq()), beta(beta), roll_offset(0_rad),
      pitch_offset(0_rad), yaw_offset(0_rad) {}

void Madgwick::calculate(const radians_per_second_t gx,
                         const radians_per_second_t gy,
                         const radians_per_second_t gz,
                         const meters_per_second_squared_t ax,
                         const meters_per_second_squared_t ay,
                         const meters_per_second_squared_t az) noexcept {
  const float _gx = gx();
  const float _gy = gy();
  const float _gz = gz();

  float _ax = ax();
  float _ay = ay();
  float _az = az();

  float recipNorm;

  // Rate of change of quaternion from gyroscope
  float qDot1 = 0.5f * (-q2 * _gx - q3 * _gy - q4 * _gz);
  float qDot2 = 0.5f * (q1 * _gx + q3 * _gz - q4 * _gy);
  float qDot3 = 0.5f * (q1 * _gy - q2 * _gz + q4 * _gx);
  float qDot4 = 0.5f * (q1 * _gz + q2 * _gy - q3 * _gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((_ax == 0.0f) && (_ay == 0.0f) && (_az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = inv_sqrt(_ax * _ax + _ay * _ay + _az * _az);
    _ax *= recipNorm;
    _ay *= recipNorm;
    _az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    const float _2q0 = 2.0f * q1;
    const float _2q1 = 2.0f * q2;
    const float _2q2 = 2.0f * q3;
    const float _2q3 = 2.0f * q4;
    const float _4q0 = 4.0f * q1;
    const float _4q1 = 4.0f * q2;
    const float _4q2 = 4.0f * q3;
    const float _8q1 = 8.0f * q2;
    const float _8q2 = 8.0f * q3;
    const float q0q0 = q1 * q1;
    const float q1q1 = q2 * q2;
    const float q2q2 = q3 * q3;
    const float q3q3 = q4 * q4;

    // Gradient decent algorithm corrective step
    float s0 = _4q0 * q2q2 + _2q2 * _ax + _4q0 * q1q1 - _2q1 * _ay;
    float s1 = _4q1 * q3q3 - _2q3 * _ax + 4.0f * q0q0 * q2 - _2q0 * _ay - _4q1 +
               _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * _az;
    float s2 = 4.0f * q0q0 * q3 + _2q0 * _ax + _4q2 * q3q3 - _2q3 * _ay - _4q2 +
               _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * _az;
    float s3 = 4.0f * q1q1 * q4 - _2q1 * _ax + 4.0f * q2q2 * q4 - _2q2 * _ay;
    recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                         s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= this->beta * s0;
    qDot2 -= this->beta * s1;
    qDot3 -= this->beta * s2;
    qDot4 -= this->beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q1 += qDot1 * (1.0f / this->sample_freq);
  q2 += qDot2 * (1.0f / this->sample_freq);
  q3 += qDot3 * (1.0f / this->sample_freq);
  q4 += qDot4 * (1.0f / this->sample_freq);

  // Normalise quaternion
  recipNorm = inv_sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  q4 *= recipNorm;
}

radian_t Madgwick::roll() const noexcept {
  const float y = 2 * q3 * q4 - 2 * q1 * q2;
  const float x = 2 * (q1 * q1) + 2 * (q4 * q4) - 1;
  const radian_t offset = this->roll_offset;
  const float rad = atan2f(y, x);
  const auto curr_roll = radian_t(rad);
  const radian_t result = offset_angle(curr_roll, offset);
  return result;
}

radian_t Madgwick::pitch() const noexcept {
  const float val = 2 * q2 * q4 + 2 * q1 * q3;
  const radian_t offset = this->pitch_offset;
  const float rad = -asinf(val);
  const auto curr_pitch = radian_t(rad);
  const radian_t result = offset_angle(curr_pitch, offset);
  return result;
}

radian_t Madgwick::yaw() const noexcept {
  const float y = 2 * q2 * q3 - 2 * q1 * q4;
  const float x = 2 * (q1 * q1) + 2 * (q2 * q2) - 1;
  const radian_t offset = this->yaw_offset;
  const float rad = atan2f(y, x);
  const auto curr_yaw = radian_t(rad);
  const radian_t result = offset_angle(curr_yaw, offset);
  return result;
}

void Madgwick::tare(const radian_t yaw, const radian_t pitch,
                    const radian_t roll) noexcept {
  this->yaw_offset = yaw - this->yaw();
  this->pitch_offset = pitch - this->pitch();
  this->roll_offset = roll - this->roll();
}

Eigen::Quaternionf Madgwick::quaternion() const noexcept {
  return {this->q1, this->q2, this->q3, this->q4};
}

// NOLINTBEGIN
inline float inv_sqrt(const float value) noexcept {
  float halfx = 0.5f * value;
  float y = value;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
// NOLINTEND

radian_t offset_angle(const radian_t angle, const radian_t offset) noexcept {
  const radian_t sum = angle + offset;
  return sum - degree_t(360) *
                   units::math::floor((sum + degree_t(180)) / degree_t(360));
}

} // namespace vilot

namespace vilot::device {

Imu::Imu(const uint8_t port, const float beta)
    : task(pros::Task::current()), inertial(port), filter(100.0_Hz, beta),
      is_calibrating(true) {
  this->task = pros::Task([this]() { this->update(); }, TASK_PRIORITY_MAX,
                          TASK_STACK_DEPTH_MIN);
}

void Imu::reset(const radian_t yaw, const radian_t pitch, const radian_t roll) {
  mut.lock();
  this->filter.tare(yaw, pitch, roll);
  mut.unlock();
}

bool Imu::start() {
  using namespace units::math;

  if (const bool inertial_success = this->inertial.reset(false);
      !inertial_success) {
    return false;
  }
  while (this->inertial.is_calibrating()) {
    pros::Task::delay(100);
  }
  this->task.notify();

  etl::circular_buffer<radian_t, 100> window;

  while (window.available()) {
    pros::Task::delay(10);
    mut.lock();
    radian_t yaw = this->filter.yaw();
    mut.unlock();
    window.push(yaw);
  }

  auto pct_diff = [&window]() {
    const auto pct =
        abs((window.front() - window.back()) / window.front()).value();
    return pct > 0.00005;
  };

  while (pct_diff()) {
    mut.lock();
    radian_t yaw = this->filter.yaw();
    mut.unlock();
    window.push(yaw);
    pros::Task::delay(10);
  }

  this->mut.lock();
  this->filter.tare();
  this->mut.unlock();

  this->is_calibrating = false;

  mut.lock();
  const bool success = !std::isnan(this->filter.yaw()());
  mut.unlock();
  return success;
}

degree_t Imu::get_heading() const {
  mut.lock();
  const auto ret = this->filter.yaw();
  mut.unlock();
  return ret;
}

degree_t Imu::get_yaw() const {
  mut.lock();
  const auto ret = this->filter.yaw();
  mut.unlock();
  return ret;
}

degree_t Imu::get_pitch() const {
  mut.lock();
  const auto ret = this->filter.pitch();
  mut.unlock();
  return ret;
}

degree_t Imu::get_roll() const {
  mut.lock();
  const auto ret = this->filter.roll();
  mut.unlock();
  return ret;
}

uint8_t Imu::get_port() const { return this->inertial.get_port(); }

[[noreturn]] void Imu::update() {
  pros::Task::notify_take(true, TIMEOUT_MAX);
  auto start = pros::millis();
  for (;;) {
    const auto [gx, gy, gz] = this->inertial.get_gyro_rate();
    const auto [ax, ay, az] = this->inertial.get_accel();

    mut.lock();
    this->filter.calculate(degrees_per_second_t(static_cast<float>(gx)),
                           degrees_per_second_t(static_cast<float>(gy)),
                           degrees_per_second_t(static_cast<float>(gz)),
                           standard_gravity_t(static_cast<float>(ax)),
                           standard_gravity_t(static_cast<float>(ay)),
                           standard_gravity_t(static_cast<float>(az)));
    mut.unlock();

    pros::Task::delay_until(&start, 10);
  }
}

} // namespace vilot::device
