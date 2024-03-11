#include "main.h"
#include "declerations.hpp"
using namespace pros;

void initialize() {
	leftMotors.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	rightMotors.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	intake.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	catapult.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	lcd::initialize();
	lcd::set_text(1, "ok program SELECTED");
	controls.set_text(0, 0, "ok program SELECTED");

	gyro.reset();
}

void disabled() {}

#include "auton.hpp"
void autonomous() {
	maybeAWP();
}

#include "opcontrol.hpp"
void opcontrol() {
	while (true) {
		leftMotors.move(controls.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + controls.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		rightMotors.move(controls.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - controls.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		intakeControl();
		catapultControl();
		frontWingsControl();
		backWingsControl();
		hangControl();

		delay(20);
	}
}