#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/drivetrain.hpp"
#include "vilot/inertial.h"
#include "vilot/localisation.hpp"
#include "vilot/ramsete.hpp"
#include "voyage/cubicspline.hpp"
#include "voyage/motionprofile.hpp"
#include <initializer_list>
#include <optional>
#include <string>

using namespace units::literals;

// vilot::device::Imu imu(7);
vilot::device::Odometry odom(7, 4, 16.5_cm, 131.9_mm, std::nullopt,
                             std::nullopt);

auto bot = vilot::drivetrain::Differential(
    12.668_in, 1.5_in, std::initializer_list<signed char>{11, -12, 13, 14},
    std::initializer_list<signed char>{-16, 17, -18, -19}, 0.8,
    pros::v5::MotorGears::blue);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  constexpr voyage::vector<5> x = {{0, 1, 2, 3, 4}};
  constexpr voyage::vector<5> y = {{1.7, -6, 5, 6.5, 0.0}};

  constexpr voyage::vector<7> x2 = {{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0}};
  constexpr voyage::vector<7> y2 = {{0.7, -6, 5, 6.5, 0.0, 5.0, -2.0}};

  constexpr voyage::CubicSpline1d spline(x, y);
  constexpr voyage::CubicSpline2d spline2d(x2, y2);

  constexpr int size = spline2d.course_size_from_step(0.1);
  constexpr auto a = spline2d.calc_course<size>();

  static_assert(a.value().first.size == size);
  static_assert(a.value().second.size == size);

  for (std::size_t i = 0; i < size; i++) {
    auto [xs, xy] = a.value();
    printf("%.6f, %.6f\n", xs[i], xy[i]);
  }

  constexpr auto final = spline(3.5);

  static_assert(final.value() >= 3.5 && final.value() < 4);

  // imu.start();
  odom.start();
  pros::Task([=]() {
    while (true) {
      printf("Heading: %f | X: %f\n",
             odom.get_state().theta.convert<units::angle::degree>().value(),
             odom.get_state().x.value());
      pros::Task::delay(500);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  auto state = odom.get_state();
  const auto start_state = state;
  auto tmp =
      voyage::TrapezoidalMotionProfile(3_m, 1.92_mps, 1.2_mps_sq, 0.9_mps_sq);
  auto rc = vilot::RamseteController(0.5, 0.05);

  units::time::millisecond_t total_ms = tmp.motion_total_time();
  units::time::millisecond_t step_size_period = 10_ms;
  int iterations = total_ms / step_size_period;

  for (int i = 0; i < iterations; i++) {
    auto t = static_cast<float>(i) * tmp.motion_total_time() / (iterations - 1);
    auto pp = tmp.sample(units::time::millisecond_t(t));
    auto [lin, ang] =
        rc.calculate({state.x(), state.y(), state.theta()},
                     {start_state.x() + pp.position(), start_state.y(), 0},
                     pp.velocity(), 0);
    bot.move(units::velocity::meters_per_second_t(lin),
             units::angular_velocity::radians_per_second_t(ang));
    state = odom.get_state();
    pros::Task::delay(10);
  }
  bot.stop();
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  auto time = pros::millis();
  while (true) {
    // pros::lcd::set_text(
    //     1,
    //     "Heading:" + std::to_string(odom.get_state()
    //                                     .theta.convert<units::angle::degree>()
    //                                     .value()));
    // pros::lcd::set_text(2, "X: " +
    // std::to_string(odom.get_state().x.value()));

    auto left_y = static_cast<float>(master.get_analog(ANALOG_LEFT_Y)) / 127.f;
    auto right_y =
        static_cast<float>(master.get_analog(ANALOG_RIGHT_Y)) / 127.f;
    bot.move(units::voltage::millivolt_t(left_y * 12000),
             units::voltage::millivolt_t(right_y * 12000));

    pros::Task::delay_until(&time, 25);
  }
}