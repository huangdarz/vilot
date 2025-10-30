#include "vilot/encoder.hpp"
#include <utility>
#include "pros/rtos.h"
#include "pros/rtos.hpp"

namespace vilot {

using namespace units::angle;
using namespace units::angular_velocity;

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

}  // namespace vilot::device