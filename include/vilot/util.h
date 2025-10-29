#pragma once
#include "Eigen/Dense"
#include "units.h"

namespace vilot {

class RobotPose2d {
  using meter_t = units::length::meter_t;
  using radian_t = units::angle::radian_t;

 public:
  RobotPose2d();
  RobotPose2d(meter_t x, meter_t y, radian_t theta);

  [[nodiscard]] meter_t x() const noexcept;
  [[nodiscard]] meter_t y() const noexcept;
  [[nodiscard]] radian_t theta() const noexcept;

  const Eigen::Isometry2f& get_transform() const;
  Eigen::Isometry2f& get_transform();

 private:
  Eigen::Isometry2f transform;
};

template <typename T>
concept Startable = requires(T t) {
  { t.start() } -> std::convertible_to<bool>;
  { t.start() } -> std::same_as<void>;
};

}  // namespace vilot