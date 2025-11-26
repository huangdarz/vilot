#pragma once

#include "drivetrain.hpp"
#include "localisation.hpp"
#include "ramsete.hpp"
#include "voyage/motionprofile.hpp"

namespace vilot {

struct NoConstraints {};

template <PoseProvider PP, bool StoreConstraints = false>
class LateralProfileController {
  using meter_t = units::length::meter_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;

 public:
  using ConstraintStorage =
      std::conditional_t<StoreConstraints,
                         voyage::TrapezoidalMotionProfile::Constraints,
                         NoConstraints>;

  LateralProfileController(DifferentialChassis& chassis, PP& localiser,
                           const voyage::TrapezoidalMotionProfile::Constraints&
                               profile_constraints) requires StoreConstraints
      : chassis(chassis),
        localiser(localiser),
        profile_constraints(std::move(profile_constraints)) {}

  LateralProfileController(DifferentialChassis& chassis, PP& localiser)
      : chassis(chassis), localiser(localiser), profile_constraints() {}

  void follow(const meter_t distance, const float follow_strength,
              const float follow_dampen) requires StoreConstraints {
    _follow_impl(distance, profile_constraints, follow_strength, follow_dampen);
  }

  void follow(const meter_t distance,
              const voyage::TrapezoidalMotionProfile::Constraints& constraints,
              const float follow_strength, const float follow_dampen)
      requires(!StoreConstraints) {
    _follow_impl(distance, constraints, follow_strength, follow_dampen);
  }

  void follow(
      const meter_t distance,
      const voyage::TrapezoidalMotionProfile::Constraints& override_constraints,
      const float follow_strength,
      const float follow_dampen) requires StoreConstraints {
    _follow_impl(distance, override_constraints, follow_strength,
                 follow_dampen);
  }

 private:
  void _follow_impl(
      const meter_t distance,
      const voyage::TrapezoidalMotionProfile::Constraints& constraints,
      const float follow_strength, const float follow_dampen) {
    using namespace units::literals;
    using namespace units::math;

    auto curr_mode_left = this->chassis.left.get_brake_mode();
    auto curr_mode_right = this->chassis.right.get_brake_mode();
    // NOLINTBEGIN
    this->chassis.left.set_brake_mode_all(
        pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    this->chassis.right.set_brake_mode_all(
        pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    // NOLINTEND

    RobotPose2d state = this->localiser.get_pose();
    const RobotPose2d start_state = state;
    voyage::TrapezoidalMotionProfile motion(abs(distance), constraints);
    RamseteController controller(follow_strength, follow_dampen);  // NOLINT
    auto prev_time = pros::millis();                               // NOLINT

    // NOLINTBEGIN
    Eigen::Vector2f unit_direction(std::cos(start_state.theta().value()),
                                   std::sin(start_state.theta().value()));
    // NOLINTEND

    motion.step_iterate(10_ms, [&](const auto& point, auto t) {
      const meter_t magnitude = copysign(point.position, distance);
      Eigen::Vector2f direction = magnitude() * unit_direction;

      const RobotPose2d desired{start_state.x() + meter_t(direction.x()),
                                start_state.y() + meter_t(direction.y()),
                                start_state.theta()};
      auto [lin, ang] = controller.calculate(
          state, desired,
          meters_per_second_t(std::copysign(point.velocity(), distance())),
          0_rps);

      this->chassis.move(lin, -ang);
      state = this->localiser.get_pose();
      pros::Task::delay_until(&prev_time, 10);
    });

    this->chassis.stop();

    this->chassis.left.set_brake_mode_all(curr_mode_left);
    this->chassis.right.set_brake_mode_all(curr_mode_right);
  }

  DifferentialChassis& chassis;
  PP& localiser;
  [[no_unique_address]] ConstraintStorage profile_constraints;
};

template <PoseProvider PP>
class RotationPidController {
  using revolutions_per_minute_t =
      units::angular_velocity::revolutions_per_minute_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;
  using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
  using radian_t = units::angle::radian_t;
  using degree_t = units::angle::degree_t;
  using millisecond_t = units::time::millisecond_t;

 public:
  RotationPidController(DifferentialChassis& chassis, PP& localiser,
                        const PidConstants& constants,
                        const SettleCondition& settle_condition,
                        const degrees_per_second_t min_speed)
      : chassis(chassis),
        localiser(localiser),
        constants(std::move(constants)),
        settle_condition(std::move(settle_condition)),
        min_speed(min_speed) {}

  bool rotate_to(const degree_t target, const millisecond_t timeout,
                 const PidConstants& constants_override,
                 SettleCondition& settle_condition_override,
                 const degrees_per_second_t min_speed_override) noexcept {
    using namespace units::math;
    using namespace units::literals;

    auto curr_mode_left = this->chassis.left.get_brake_mode();
    auto curr_mode_right = this->chassis.right.get_brake_mode();
    // NOLINTBEGIN
    this->chassis.left.set_brake_mode_all(
        pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    this->chassis.right.set_brake_mode_all(
        pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    // NOLINTEND

    PidController controller(constants_override);
    controller.set_continuous_input(true);
    controller.set_max_input(180);
    controller.set_min_input(-180);
    controller.set_abs_max_output(MAX_VOLTAGE_MV);

    auto prev_time = pros::millis();

    RobotPose2d state = this->localiser.get_pose();
    const auto start_time = pros::millis();
    while (pros::millis() - start_time < timeout() &&
           !settle_condition_override.check(
               state.theta().convert<units::angle::degrees>()(), target())) {
      RobotPose2d current_state = this->localiser.get_pose();
      const auto ang = controller.calculate(
          current_state.theta().convert<units::angle::degrees>()(), target());
      const auto turn = degrees_per_second_t(-ang);
      const degrees_per_second_t min_turn =
          fmax(abs(turn), abs(min_speed_override));
      const degrees_per_second_t val = copysign(min_turn, turn);
      this->chassis.move(0_mps, val);
      state = this->localiser.get_pose();
      pros::Task::delay_until(&prev_time, 10);
    }
    this->chassis.stop();

    this->chassis.left.set_brake_mode_all(curr_mode_left);
    this->chassis.right.set_brake_mode_all(curr_mode_right);

    return pros::millis() - start_time <= timeout();
  }

  bool rotate_to(const degree_t target, const millisecond_t timeout) {
    return this->rotate_to(target, timeout, this->constants,
                           this->settle_condition, this->min_speed);
  }

 private:
  DifferentialChassis& chassis;
  PP& localiser;
  PidConstants constants;
  SettleCondition settle_condition;
  degrees_per_second_t min_speed;
};

}  // namespace vilot