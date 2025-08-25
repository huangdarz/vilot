#include "vilot/drivetrain.hpp"

namespace vilot::drivetrain {

using namespace units::angular_velocity;

void Differential::move(meters_per_second_t x, radians_per_second_t theta) {
  float left_vel = x() + (theta() * this->track_width() / 2.0f);
  float right_vel = x() - (theta() * this->track_width() / 2.0f);

  revolutions_per_minute_t left_rot = revolutions_per_minute_t(
      units::convert<radians_per_second, revolutions_per_minute>(
          left_vel / this->wheel_radius()));
  revolutions_per_minute_t right_rot = revolutions_per_minute_t(
      units::convert<radians_per_second, revolutions_per_minute>(
          right_vel / this->wheel_radius()));

  this->chassis.left.move_velocity(left_rot() *
                                   this->chassis.gear_ratio_in_out);
  this->chassis.right.move_velocity(right_rot() *
                                    this->chassis.gear_ratio_in_out);
}

} // namespace vilot::drivetrain