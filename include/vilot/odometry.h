#pragma once

#include "Eigen/Dense"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/inertial.h"
#include "vilot/rotation.h"
#include "vilot/util.h"
#include <cstdint>
#include <optional>

namespace vilot {

class DeadReckoning {
  using degree_t = units::angle::degree_t;
  using radian_t = units::angle::radian_t;
  using millimeter_t = units::length::millimeter_t;

public:
  DeadReckoning(millimeter_t centre_displacement, millimeter_t middle_distance,
                millimeter_t wheel_circumference);

  void update(radian_t yaw, degree_t parallel, degree_t perpendicular);

  Pose2d get_pose();

private:
  const millimeter_t centre_displacement;
  const millimeter_t middle_distance;
  const millimeter_t wheel_circumference;

  Eigen::Vector2f position;
  degree_t heading;

  millimeter_t prev_parallel;
  millimeter_t prev_perpendicular;
};

} // namespace vilot

namespace vilot::device {

class Odometry {
  using millimeter_t = units::length::millimeter_t;
  using degree_t = units::angle::degree_t;

public:
  Odometry(uint8_t imu, int8_t parallel, millimeter_t centre_displacement,
           millimeter_t wheel_circumference,
           std::optional<int8_t> perpendicular,
           std::optional<millimeter_t> middle_distance);

  void start();

  Pose2d get_pose();

private:
  void update();

  Imu imu;
  Rotation parallel;
  std::optional<Rotation> perpendicular;
  pros::MutexVar<DeadReckoning> dead_reckoning;

  pros::Task task;
};

} // namespace vilot::device