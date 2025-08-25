#pragma once
#include "Eigen/Dense"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/inertial.h"
#include "vilot/rotation.h"
#include <concepts>

namespace vilot::localisation {

struct ChassisState {
  using meter_t = units::length::meter_t;
  using radian_t = units::angle::radian_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;

  // Global frame
  meter_t x;
  meter_t y;
  radian_t theta;

  // Local frame
  meters_per_second_t vx;
};

template <typename T>
concept Localiser = requires(T t) {
  { t.get_state() } -> std::same_as<ChassisState>;
};

struct BaseLocalisation {
  virtual ChassisState get_state() = 0;
};

class DeadReckoning {
  using degree_t = units::angle::degree_t;
  using radian_t = units::angle::radian_t;
  using millimeter_t = units::length::millimeter_t;
  using meter_t = units::length::meter_t;
  using second_t = units::time::second_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;

public:
  DeadReckoning() = delete;
  DeadReckoning(millimeter_t centre_displacement, millimeter_t middle_distance,
                millimeter_t wheel_circumference);

  void update(radian_t yaw, degree_t parallel, degree_t perpendicular,
              second_t dt);

  ChassisState get_state();

private:
  const meter_t centre_displacement;
  const meter_t middle_distance;
  const meter_t wheel_circumference;

  Eigen::Vector2f position;
  degree_t heading;
  meters_per_second_t velocity;

  meter_t prev_parallel;
  meter_t prev_perpendicular;
};

} // namespace vilot::localisation

namespace vilot::device {

class Odometry : public localisation::BaseLocalisation {
  using millimeter_t = units::length::millimeter_t;
  using degree_t = units::angle::degree_t;
  using radian_t = units::angle::radian_t;

public:
  Odometry(uint8_t imu, int8_t parallel, millimeter_t centre_displacement,
           millimeter_t wheel_circumference,
           std::optional<int8_t> perpendicular,
           std::optional<millimeter_t> middle_distance);

  void start();

  localisation::ChassisState get_state() override;

private:
  void update();

  Imu imu;
  Rotation parallel;
  std::optional<Rotation> perpendicular;
  pros::MutexVar<localisation::DeadReckoning> dead_reckoning;

  pros::Task task;
};

} // namespace vilot::device