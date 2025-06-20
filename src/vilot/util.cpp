#include "vilot/util.h"

namespace vilot {

Pose2d::Pose2d(Eigen::Vector2f vec, radian_t theta)
    : x(meter_t(vec.x())), y(meter_t(vec.y())), theta(theta) {}

Pose2d::Pose2d(meter_t x, meter_t y, radian_t theta)
    : x(x), y(y), theta(theta) {}

Eigen::Vector2f Pose2d::as_vec() { return Eigen::Vector2f{x, y}; }

Twist2d::Twist2d(Eigen::Vector2f vec, radians_per_second_t theta)
    : x(meters_per_second_t(vec.x())), y(meters_per_second_t(vec.y())),
      theta(theta) {}

Twist2d::Twist2d(meters_per_second_t x, meters_per_second_t y,
                 radians_per_second_t theta)
    : x(x), y(y), theta(theta) {}

Eigen::Vector2f Twist2d::as_vec() { return Eigen::Vector2f{x, y}; }

} // namespace vilot