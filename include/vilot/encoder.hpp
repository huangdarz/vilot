#pragma once
#include <utility>
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/filter.hpp"
#include "vilot/util.hpp"

namespace vilot {

template <typename T>
concept RotationEncoderProvider = requires(T t, const T& ct) {
  { t.tare() } -> std::same_as<void>;
  { ct.get_position() } -> std::convertible_to<units::angle::degree_t>;
  {
    ct.get_velocity()
  } -> std::convertible_to<units::angular_velocity::degrees_per_second_t>;
};

class Encoder {
  using degree_t = units::angle::degree_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

 public:
  template <typename... Args>
  explicit Encoder(Args... args)
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

class QuadratureEncoder {
  using degree_t = units::angle::degree_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

 public:
  template <typename... Args>
  explicit QuadratureEncoder(Args... args)
      : encoder(std::forward<Args>(args)...),
        pos_filter(100, 50),
        vel_filter(100, 50),
        previous_position(0) {}

  std::pair<degree_t, degrees_per_second_t> update(
      units::time::millisecond_t dt);
  void reset();

  pros::adi::Encoder encoder;

 private:
  BiquadLowPassFilter pos_filter;
  BiquadLowPassFilter vel_filter;
  int32_t previous_position;
};

}  // namespace vilot

namespace vilot::device {

class Rotation {
  using degree_t = units::angle::degree_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

 public:
  template <typename... Args>
  explicit Rotation(int8_t port)
      : encoder(port), task(pros::Task::current()), position(0), velocity(0) {
    this->task = pros::Task([this]() { this->update(); });
  }

  void start();

  void tare();
  degree_t get_position() const;
  degrees_per_second_t get_velocity() const;

 private:
  void update();

  Encoder encoder;
  pros::Task task;
  mutable pros::MutexVar<degree_t> position;
  mutable pros::MutexVar<degrees_per_second_t> velocity;
};

class Encoder {
  using degree_t = units::angle::degree_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

 public:
  template <typename... Args>
  explicit Encoder(Args... args)
      : encoder(std::forward<Args>(args)...),
        task(pros::Task::current()),
        position(0),
        velocity(0) {
    this->task = pros::Task([this]() { this->update(); });
  }

  void start();
  void tare();
  degree_t get_position() const;
  degrees_per_second_t get_velocity() const;

 private:
  [[noreturn]] void update();

  QuadratureEncoder encoder;
  pros::Task task;
  mutable pros::MutexVar<degree_t> position;
  mutable pros::MutexVar<degrees_per_second_t> velocity;
};

static_assert(vilot::RotationEncoderProvider<Rotation>);
static_assert(vilot::Startable<Rotation>);

static_assert(vilot::RotationEncoderProvider<Encoder>);
static_assert(vilot::Startable<Encoder>);

}  // namespace vilot::device