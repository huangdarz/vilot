#include "vilot/encoder.hpp"
#include <utility>
#include "pros/rtos.h"
#include "pros/rtos.hpp"

namespace vilot {

using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::time;
using namespace units::literals;

std::pair<degree_t, degrees_per_second_t> Encoder::update() {
  int32_t pos = this->rotation.get_position();
  int32_t vel = this->rotation.get_velocity();
  auto new_pos = degree_t(this->pos_filter.apply((float)pos) / 100.0f);
  auto new_vel =
      degrees_per_second_t(this->vel_filter.apply((float)vel) / 100.0f);
  return std::pair<degree_t, degrees_per_second_t>{new_pos, new_vel};
}

void Encoder::reset() {
  this->pos_filter.reset();
  this->vel_filter.reset();
  this->rotation.reset();
}

std::pair<degree_t, degrees_per_second_t> QuadratureEncoder::update(
    const millisecond_t dt) {
  const int32_t pos = this->encoder.get_value();
  const float vel = static_cast<float>(pos - this->previous_position) /
                    (dt.convert<second>().value());
  auto new_pos = degree_t(
      this->pos_filter.apply((static_cast<float>(pos)) / 4096.0f) * 360.f);
  auto new_vel =
      degrees_per_second_t((this->vel_filter.apply(vel) / 4096.f) * 360.f);
  return std::pair<degree_t, degrees_per_second_t>{new_pos, new_vel};
}

void QuadratureEncoder::reset() {
  this->pos_filter.reset();
  this->vel_filter.reset();
  this->encoder.reset();  // NOLINT
}

}  // namespace vilot

namespace vilot::device {

using namespace units::angle;
using namespace units::angular_velocity;

void Rotation::start() {
  this->task.notify();
}

void Rotation::update() {
  this->task.notify_take(true, TIMEOUT_MAX);
  auto start = pros::millis();
  for (;;) {
    auto [pos, vel] = this->encoder.update();
    *this->position.lock() = pos;
    *this->velocity.lock() = vel;
    pros::Task::delay_until(&start, 10);
  }
}

void Rotation::tare() {
  this->encoder.reset();
}

degree_t Rotation::get_position() const {
  return *this->position.lock();
}

degrees_per_second_t Rotation::get_velocity() const {
  return *this->velocity.lock();
}

void Encoder::start() {
  this->task.notify();
}

[[noreturn]] void Encoder::update() {
  this->task.notify_take(true, TIMEOUT_MAX);
  auto start = pros::millis();
  for (;;) {
    auto [pos, vel] = this->encoder.update(10_ms);
    *this->position.lock() = pos;
    *this->velocity.lock() = vel;
    pros::Task::delay_until(&start, 10);
  }
}

void Encoder::tare() {
  this->encoder.reset();
}

degree_t Encoder::get_position() const {
  return *this->position.lock();
}

degrees_per_second_t Encoder::get_velocity() const {
  return *this->velocity.lock();
}

}  // namespace vilot::device