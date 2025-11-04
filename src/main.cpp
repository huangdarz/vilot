#include "main.h"
#include <string>
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/drivetrain.hpp"
#include "vilot/localisation.hpp"

using namespace units::literals;
using namespace vilot;

device::Imu imu(7);
device::Rotation rot(4);
Odometry odom(imu, rot, 16.5_mm, 131.9_mm);

DifferentialChassis chassis(PORTS(11, -12, 13, 14), PORTS(-16, 17, -18, -19),
                            1.25, 12.668_in, 1.5_in,
                            pros::v5::MotorGears::blue);

pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  static_assert(vilot::OrientationProvider<device::Imu>);
  static_assert(vilot::RotationEncoderProvider<device::Rotation>);
  static_assert(vilot::Startable<device::Imu>);
  static_assert(vilot::Startable<device::Rotation>);

  odom.start();
  // imu.start();
  // bool start_success = odom.start();
  // if (start_success) {
  //   master.rumble(". .");
  // } else {
  //   master.rumble("- - -");
  // }
  // pros::Task([=]() {
  //   while (true) {
  //     auto state = odom.get_state();
  //     printf("Heading: %f | X: %f\n",
  //            state.pose.theta.convert<units::angle::degree>().value(),
  //            state.pose.x.value());
  //     pros::Task::delay(500);
  //   }
  // });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // vilot::SettleCondition cond{
  //     .condition = [](float m, float g) { return fabs(m - g) < 0.5; },
  //     .settle_time = 400};
  // vilot::PidConstants kon(1, 0, 0);
  //
  // bot.follow(2_m, 1.92_mps, 1.5_mps_sq, 0.6_mps_sq, 3.2, 0.05);
  // bot.rotate_to(90_deg, kon, cond, 5_deg_per_s, 10000_ms);
  // bot.follow(2_m, 1.92_mps, 1.5_mps_sq, 0.6_mps_sq, 3.2, 0.05);
  // bot.rotate_to(90_deg, kon, cond, 5_deg_per_s, 10000_ms);

  master.rumble(".");
}

void opcontrol() {
  auto time = pros::millis();
  while (true) {
    // pros::lcd::set_text(
    //     1,
    //     "Heading:" + std::to_string(odom.get_state()
    //                                     .theta.convert<units::angle::degree>()
    //                                     .value()));
    // pros::lcd::set_text(2, "X: " +
    // std::to_string(odom.get_state().x.value()));

    // auto left_y = static_cast<float>(master.get_analog(ANALOG_LEFT_Y)) / 127.f;
    // auto right_x =
    //     static_cast<float>(master.get_analog(ANALOG_RIGHT_X)) / 127.f;
    // bot.move(units::voltage::millivolt_t(left_y * 12000),
    //          units::voltage::millivolt_t(right_x * 12000));

    pros::Task::delay_until(&time, 25);
  }
}
