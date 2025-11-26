#include "vilot/localisation.hpp"
#include <cmath>
#include "pros/rtos.h"
#include "units.h"

namespace vilot {

using namespace units::time;
using namespace units::literals;

DeadReckoning::DeadReckoning(const millimeter_t centre_displacement,
                             const millimeter_t middle_distance,
                             const millimeter_t wheel_circumference)
    : centre_displacement(centre_displacement),
      middle_distance(middle_distance),
      wheel_circumference(wheel_circumference),
      heading(0_deg),
      velocity(0_mps),
      prev_parallel(0_m),
      prev_perpendicular(0_m) {}

void DeadReckoning::update(const radian_t yaw, const degree_t parallel,
                           const degree_t perpendicular, const second_t dt) {
  const auto parallel_displacement =
      meter_t(parallel() / (360.0 / this->wheel_circumference()));
  const meter_t perpendicular_displacement =
      this->middle_distance() == 0.0
          ? meter_t(0.0)
          : meter_t(perpendicular() / (360.0 / this->middle_distance()));

  const meter_t delta_parallel_displacement =
      parallel_displacement - this->prev_parallel;
  this->prev_parallel = parallel_displacement;

  meter_t delta_perpendicular_displacement =
      perpendicular_displacement - this->prev_perpendicular;
  this->prev_perpendicular = perpendicular_displacement;

  const radian_t delta_theta = yaw - this->heading;
  this->heading = degree_t(yaw);

  Eigen::Vector2f local_displacement(0, 0);

  if (!std::isnan(delta_theta()) && delta_theta() != 0.0) {
    const float scalar = std::sin(delta_theta() / 2.0) * 2.0;
    local_displacement = {(delta_parallel_displacement() / delta_theta() -
                           this->centre_displacement()) *
                              scalar,
                          (delta_perpendicular_displacement() / delta_theta()) +
                              middle_distance() * scalar};
  } else {
    local_displacement = {delta_parallel_displacement(),
                          delta_perpendicular_displacement};
  }

  const float p = yaw() - delta_theta() / 2.0;
  const Eigen::Rotation2Df rot(p);
  const Eigen::Vector2f global_displacement = rot * local_displacement;

  this->position += global_displacement;
  this->velocity = parallel_displacement / dt;
}

RobotPose2d DeadReckoning::get_state() const {
  return {meter_t(this->position.x()), meter_t(this->position.y()),
          radian_t(this->heading)};
}

void DeadReckoning::tare(meter_t x, meter_t y) {
  this->position = Eigen::Vector2f{x(), y()};
  this->velocity = meters_per_second_t(0);
}

}  // namespace vilot