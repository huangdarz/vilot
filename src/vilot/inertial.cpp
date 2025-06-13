#include "vilot/inertial.h"
#include "etl/circular_buffer.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

namespace vilot {

using namespace units::literals;
using namespace units::angle;
using namespace units::acceleration;
using namespace units::angular_velocity;

Madgwick::Madgwick(const hertz_t sample_freq, const float beta)
    : sample_freq(sample_freq()), beta(beta), roll_offset(0_rad),
      pitch_offset(0_rad), yaw_offset(0_rad) {}

void Madgwick::calculate(const radians_per_second_t _gx,
                         const radians_per_second_t _gy,
                         const radians_per_second_t _gz,
                         const meters_per_second_squared_t _ax,
                         const meters_per_second_squared_t _ay,
                         const meters_per_second_squared_t _az) noexcept {
  const float gx = _gx();
  const float gy = _gy();
  const float gz = _gz();

  float ax = _ax();
  float ay = _ay();
  float az = _az();

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q1;
    _2q1 = 2.0f * q2;
    _2q2 = 2.0f * q3;
    _2q3 = 2.0f * q4;
    _4q0 = 4.0f * q1;
    _4q1 = 4.0f * q2;
    _4q2 = 4.0f * q3;
    _8q1 = 8.0f * q2;
    _8q2 = 8.0f * q3;
    q0q0 = q1 * q1;
    q1q1 = q2 * q2;
    q2q2 = q3 * q3;
    q3q3 = q4 * q4;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q2 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q3 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q4 - _2q1 * ax + 4.0f * q2q2 * q4 - _2q2 * ay;
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

void Madgwick::operator()(const radians_per_second_t _gx,
                          const radians_per_second_t _gy,
                          const radians_per_second_t _gz,
                          const meters_per_second_squared_t _ax,
                          const meters_per_second_squared_t _ay,
                          const meters_per_second_squared_t _az) noexcept {
  this->calculate(_gx, _gy, _gz, _ax, _ay, _az);
}

const radian_t Madgwick::roll() const noexcept {
  const float y = 2 * q3 * q4 - 2 * q1 * q2;
  const float x = 2 * (q1 * q1) + 2 * (q4 * q4) - 1;
  const radian_t offset = this->roll_offset;
  const float rad = atan2f(y, x);
  const radian_t curr_roll = radian_t(rad);
  const radian_t result = offset_angle(curr_roll, offset);
  return result;
}

const radian_t Madgwick::pitch() const noexcept {
  const float val = 2 * q2 * q4 + 2 * q1 * q3;
  const radian_t offset = this->pitch_offset;
  const float rad = -asinhf(val);
  const radian_t curr_pitch = radian_t(rad);
  const radian_t result = offset_angle(curr_pitch, offset);
  return result;
}

const radian_t Madgwick::yaw() const noexcept {
  const float y = 2 * q2 * q3 - 2 * q1 * q4;
  const float x = 2 * (q1 * q1) + 2 * (q2 * q2) - 1;
  const radian_t offset = this->yaw_offset;
  const float rad = atan2f(y, x);
  const radian_t curr_yaw = radian_t(rad);
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
  return Eigen::Quaternionf(this->q1, this->q2, this->q3, this->q4);
}

const inline float inv_sqrt(const float value) noexcept {
  float halfx = 0.5f * value;
  float y = value;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

radian_t offset_angle(radian_t angle, radian_t offset) noexcept {
  radian_t sum = angle + offset;
  return sum - degree_t(360) *
                   units::math::floor((sum + degree_t(180)) / degree_t(360));
}

} // namespace vilot

namespace vilot::device {

Imu::Imu(const uint8_t port, const float beta)
    : task(pros::Task::current()), inertial(port), filter(100.0_Hz, beta),
      is_calibrating(true) {
  this->task = pros::Task([this]() { this->update(); });
}

void Imu::reset(const radian_t yaw, const radian_t pitch, const radian_t roll) {
  mut.lock();
  this->filter.tare(yaw, pitch, roll);
  mut.unlock();
}

void Imu::start() {
  using namespace units::math;

  this->inertial.reset(true);
  this->task.notify();

  etl::circular_buffer<radian_t, 100> window;

  while (window.available()) {
    mut.lock();
    radian_t yaw = this->filter.yaw();
    mut.unlock();
    window.push(yaw);
    pros::Task::delay(10);
  }

  auto pct_diff = [&window]() {
    auto pct = abs((window.front() - window.back()) / window.front()).value();
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
}

degree_t Imu::get_heading() const {
  mut.lock();
  auto ret = this->filter.yaw();
  mut.unlock();
  return ret;
}

uint8_t Imu::get_port() const { return this->inertial.get_port(); }

void Imu::update() {
  this->task.notify_take(true, TIMEOUT_MAX);
  auto start = pros::millis();
  for (;;) {
    const auto g = this->inertial.get_gyro_rate();
    const auto a = this->inertial.get_accel();

    mut.lock();
    this->filter.calculate(degrees_per_second_t(g.x), degrees_per_second_t(g.y),
                           degrees_per_second_t(g.z), standard_gravity_t(a.x),
                           standard_gravity_t(a.y), standard_gravity_t(a.z));
    mut.unlock();

    pros::Task::delay_until(&start, 10);
  }
}

} // namespace vilot::device