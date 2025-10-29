#pragma once
#include <utility>
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/filter.h"

namespace vilot {

template <typename T>
concept RotationEncoderProvider = requires(T t, const T& ct) {
  { t.tare() } -> std::same_as<void>;
  { ct.get_position() } -> std::convertible_to<units::angle::degree_t>;
  {
    ct.get_velocity()
  } -> std::same_as<units::angular_velocity::degrees_per_second>;
};

class Encoder {
  using degree_t = units::angle::degree_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

 public:
  template <typename... Args>
  Encoder(Args... args)
      : rotation(std::forward<Args>(args)...),
        pos_filter(100, 50),
        vel_filter(100, 50) {}

  std::pair<degree_t, degrees_per_second_t> update();
  void reset();

  pros::Rotation rotation;

 private:
  BiquadLowPassFilter pos_filter;
  BiquadLowPassFilter vel_filter;
};

}  // namespace vilot

namespace vilot::device {

class Rotation {
  using degree_t = units::angle::degree_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

 public:
  template <typename... Args>
  Rotation(int8_t port)
      : encoder(port), task(pros::Task::current()), position(0), velocity(0) {
    this->task = pros::Task([this]() { this->update(); });
  }

  void start();

  degree_t get_position();
  degrees_per_second_t get_velocity();

 private:
  void update();

  Encoder encoder;
  pros::Task task;
  pros::MutexVar<degree_t> position;
  pros::MutexVar<degrees_per_second_t> velocity;
};

}  // namespace vilot::device