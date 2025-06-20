#include "vilot/odometry.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units.h"
#include "util.h"

namespace vilot {

using namespace units::angle;
using namespace units::length;

DeadReckoning::DeadReckoning(millimeter_t centre_displacement,
                             millimeter_t middle_distance,
                             millimeter_t wheel_circumference)
    : centre_displacement(centre_displacement),
      middle_distance(middle_distance),
      wheel_circumference(wheel_circumference) {}

void DeadReckoning::update(radian_t yaw, degree_t parallel,
                           degree_t perpendicular) {
  millimeter_t parallel_displacement =
      millimeter_t(parallel() / (360.0 / this->wheel_circumference()));
  millimeter_t perpendicular_displacement =
      this->middle_distance()
          ? millimeter_t(0.0)
          : millimeter_t(perpendicular() / (360.0 / this->middle_distance()));

  millimeter_t delta_parallel_displacement =
      parallel_displacement - this->prev_parallel;
  this->prev_parallel = parallel_displacement;

  millimeter_t delta_perpendicular_displacement =
      perpendicular_displacement - this->prev_perpendicular;
  this->prev_perpendicular = perpendicular_displacement;

  radian_t delta_theta = yaw - this->heading;
  this->heading = degree_t(yaw);

  Eigen::Vector2f local_displacement(0, 0);

  if (delta_theta()) {
    float scalar = std::sin(delta_theta() / 2.0) * 2.0;
    local_displacement = {(delta_parallel_displacement() / delta_theta() -
                           this->centre_displacement()) *
                              scalar,
                          (delta_perpendicular_displacement() / delta_theta()) +
                              middle_distance() * scalar};
  } else {
    local_displacement = {delta_parallel_displacement(),
                          delta_perpendicular_displacement};
  }

  float p = yaw() - delta_theta() / 2.0;
  Eigen::Rotation2Df rot(p);
  Eigen::Vector2f global_displacement = rot * local_displacement;

  this->position += global_displacement;
}

Pose2d DeadReckoning::get_pose() {
  return Pose2d(this->position, this->heading);
}

} // namespace vilot

namespace vilot::device {

Odometry::Odometry(uint8_t imu, int8_t parallel,
                   millimeter_t centre_displacement,
                   millimeter_t wheel_circumference,
                   std::optional<int8_t> perpendicular,
                   std::optional<millimeter_t> middle_distance)
    : imu(imu), parallel(parallel), perpendicular(perpendicular),
      dead_reckoning(centre_displacement,
                     middle_distance.value_or(millimeter_t(0)),
                     wheel_circumference),
      task(pros::Task::current()) {
  this->task = pros::Task([this]() { this->update(); });
}

Pose2d Odometry::get_pose() { return this->dead_reckoning.lock()->get_pose(); }

void Odometry::start() {
  this->imu.start();
  this->parallel.start();
  if (perpendicular.has_value()) {
    perpendicular->start();
  }
  pros::Task::delay(40);
  this->task.notify();
}

void Odometry::update() {
  this->task.notify_take(true, TIMEOUT_MAX);
  auto start = pros::millis();
  for (;;) {
    radian_t yaw = this->imu.get_heading();
    degree_t parallel_pos = parallel.get_position();
    degree_t perpendicular_pos =
        perpendicular.has_value() ? perpendicular->get_position() : degree_t(0);
    this->dead_reckoning.lock()->update(yaw, parallel_pos, perpendicular_pos);
    pros::Task::delay_until(&start, 10);
  }
}

} // namespace vilot::device