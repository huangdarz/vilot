#include "vilot/util.h"

namespace vilot {

using namespace units::length;
using namespace units::angle;

RobotPose2d::RobotPose2d() : transform(Eigen::Isometry2f::Identity()) {}

RobotPose2d::RobotPose2d(const meter_t x, const meter_t y,
                         const radian_t theta) {
  this->transform =
      Eigen::Translation2f(x(), y()) * Eigen::Rotation2Df(theta());
}

meter_t RobotPose2d::x() const noexcept {
  return meter_t(this->transform.translation().x());
}

meter_t RobotPose2d::y() const noexcept {
  return meter_t(this->transform.translation().y());
}

radian_t RobotPose2d::theta() const noexcept {
  const auto rot = this->transform.rotation();
  return radian_t(std::atan2f(rot(1, 0), rot(0, 0)));
}

const Eigen::Isometry2f& RobotPose2d::get_transform() const {
  return this->transform;
}

Eigen::Isometry2f& RobotPose2d::get_transform() {
  return this->transform;
}

}  // namespace vilot