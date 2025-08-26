#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "vilot/inertial.h"
#include "voyage/cubicspline.hpp"
#include <string>

vilot::device::Imu imu(7);

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
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  imu.start();
  auto time = pros::millis();
  while (true) {
    pros::lcd::set_text(1, "Yaw: " + std::to_string(imu.get_heading().value()));
    pros::lcd::set_text(2, "Roll: " + std::to_string(imu.get_roll().value()));
    pros::lcd::set_text(3, "Pitch: " + std::to_string(imu.get_pitch().value()));
    pros::lcd::set_text(4, "Yaw: " + std::to_string(imu.get_heading().value()));
    pros::lcd::set_text(5, "Roll: " + std::to_string(imu.get_roll().value()));
    pros::lcd::set_text(6, "Pitch: " + std::to_string(imu.get_pitch().value()));
    pros::Task::delay_until(&time, 50);
  }
}