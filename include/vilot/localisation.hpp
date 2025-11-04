#pragma once
#include "Eigen/Dense"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/encoder.hpp"
#include "vilot/inertial.hpp"
#include "vilot/util.hpp"

namespace vilot {

template <typename T>
concept PoseProvider = requires(T t, const T& ct, const RobotPose2d& pose) {
  { t.tare(pose) } -> std::same_as<void>;
  { ct.get_pose() } -> std::convertible_to<RobotPose2d>;
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

  void tare(meter_t x, meter_t y);

  RobotPose2d get_state() const;

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

template <OrientationProvider OP, RotationEncoderProvider REP_PARALLEL,
          RotationEncoderProvider REP_PERPENDICULAR = REP_PARALLEL>
class Odometry {
  using millimeter_t = units::length::millimeter_t;
  using degree_t = units::angle::degree_t;
  using radian_t = units::angle::radian_t;

 public:
  Odometry() = delete;

  Odometry(OP& orientation, REP_PARALLEL& parallel,
           millimeter_t centre_displacement, millimeter_t wheel_circumference,
           REP_PERPENDICULAR& perpendicular, millimeter_t middle_distance)
      : orientation(orientation),
        parallel(parallel),
        perpendicular(perpendicular),
        dead_reckoning(centre_displacement, middle_distance,
                       wheel_circumference),
        task(pros::Task::current()) {
    this->task =
        pros::Task([this]() { this->update(); }, TASK_PRIORITY_MAX - 1);
  }

  Odometry(OP& orientation, REP_PARALLEL& parallel,
           millimeter_t centre_displacement, millimeter_t wheel_circumference)
      : orientation(orientation),
        parallel(parallel),
        dead_reckoning(centre_displacement, millimeter_t(0),
                       wheel_circumference),
        task(pros::Task::current()) {
    this->task =
        pros::Task([this]() { this->update(); }, TASK_PRIORITY_MAX - 1);
  }

  bool start() {
    if constexpr (Startable<OP>) {
      if (const bool start_success = this->orientation.start();
          !start_success) {
        return false;
      }
    }
    if constexpr (Startable<REP_PARALLEL>) {
      this->parallel.start();
    }
    if constexpr (Startable<REP_PERPENDICULAR>) {
      if (this->perpendicular.has_value()) {
        this->perpendicular->start();
      }
    }
    pros::Task::delay(500);
    this->task.notify();
    return true;
  }

  void tare(const RobotPose2d& pose) {
    this->dead_reckoning.lock()->tare(pose.x(), pose.y());
    this->orientation.tare(pose.theta(), radian_t(0), radian_t(0));
  }

  RobotPose2d get_pose() const noexcept {
    return this->dead_reckoning.lock()->get_state();
  }

 private:
  [[noreturn]] void update() const {
    pros::Task::notify_take(true, TIMEOUT_MAX);
    auto start = pros::millis();
    for (;;) {
      radian_t yaw = this->orientation.get_yaw();
      degree_t parallel_pos = parallel.get_position();
      degree_t perpendicular_pos = perpendicular.has_value()
                                       ? perpendicular->get_position()
                                       : degree_t(0);
      this->dead_reckoning.lock()->update(yaw, parallel_pos, perpendicular_pos,
                                          units::time::millisecond_t(10));
      pros::Task::delay_until(&start, 10);
    }
  }

  OP& orientation;
  REP_PARALLEL& parallel;
  std::optional<REP_PERPENDICULAR> perpendicular;
  mutable pros::MutexVar<DeadReckoning> dead_reckoning;
  pros::Task task;
};

static_assert(PoseProvider<Odometry<device::Imu, device::Rotation>>);
static_assert(Startable<Odometry<device::Imu, device::Rotation>>);

}  // namespace vilot