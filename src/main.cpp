#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/drivetrain.hpp"
#include "vilot/localisation.hpp"
#include <string>

using namespace units::literals;

// vilot::device::Imu imu(7);
vilot::device::Odometry odom(7, 4, 16.5_cm, 131.9_mm);

auto bot = vilot::DifferentialDrivetrain(
    12.668_in, 1.5_in, odom, MOTORS(11, -12, 13, 14), MOTORS(-16, 17, -18, -19),
    0.8, pros::v5::MotorGears::blue);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  // imu.start();
  odom.start();
  pros::Task([=]() {
    while (true) {
      auto state = odom.get_state();
      printf("Heading: %f | X: %f\n",
             state.pose.theta.convert<units::angle::degree>().value(),
             state.pose.x.value());
      pros::Task::delay(500);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  bot.follow(3_m, 1.92_mps, 1.2_mps_sq, 0.9_mps_sq, 0.5, 0.05);
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
    auto right_x =
        static_cast<float>(master.get_analog(ANALOG_RIGHT_X)) / 127.f;
    bot.move(units::voltage::millivolt_t(left_y * 12000),
             units::voltage::millivolt_t(right_x * 12000));

    pros::Task::delay_until(&time, 25);
  }
}