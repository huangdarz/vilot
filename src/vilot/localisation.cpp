#include "vilot/localisation.hpp"
#include "pros/rtos.h"
#include "units.h"

namespace vilot::localisation {

using namespace units::time;

DeadReckoning::DeadReckoning(millimeter_t centre_displacement,
                             millimeter_t middle_distance,
                             millimeter_t wheel_circumference)
    : centre_displacement(centre_displacement),
      middle_distance(middle_distance),
      wheel_circumference(wheel_circumference) {}

void DeadReckoning::update(radian_t yaw, degree_t parallel,
                           degree_t perpendicular, second_t dt) {
  meter_t parallel_displacement =
      meter_t(parallel() / (360.0 / this->wheel_circumference()));
  meter_t perpendicular_displacement =
      this->middle_distance()
          ? meter_t(0.0)
          : meter_t(perpendicular() / (360.0 / this->middle_distance()));

  meter_t delta_parallel_displacement =
      parallel_displacement - this->prev_parallel;
  this->prev_parallel = parallel_displacement;

  meter_t delta_perpendicular_displacement =
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
  this->velocity = parallel_displacement / dt;
}

ChassisState DeadReckoning::get_state() {
  return {meter_t(this->position.x()), meter_t(this->position.y()),
          radian_t(this->heading), this->velocity};
}

} // namespace vilot::localisation

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
  this->task = pros::Task([this]() { this->update(); }, TASK_PRIORITY_MAX - 1);
}

localisation::ChassisState Odometry::get_state() {
  return this->dead_reckoning.lock()->get_state();
}

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
    this->dead_reckoning.lock()->update(yaw, parallel_pos, perpendicular_pos,
                                        units::time::millisecond_t(10));
    pros::Task::delay_until(&start, 10);
  }
}

} // namespace vilot::device