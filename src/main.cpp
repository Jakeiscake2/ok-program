#include "main.h"
#include "declerations.hpp"
#include "pid.hpp"
#include "auton.hpp"
using namespace pros;

void initialize() {
	frontLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	midLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	backLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	frontRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	midRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	backRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	catapult.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake.set_brake_modes(E_MOTOR_BRAKE_HOLD);

	lcd::initialize();
	lcd::set_text(1, "defensive SELECTED");
	controls.set_text(0, 0, "defensive SELECTED");
	gyro.reset();
}

void disabled() {
}
void competition_initialize() {}

void autonomous() {
	maybeAWP();
}

void opcontrol() {
	catapult.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	catapult.tare_position();
	int x1, y1 = 0;
	int line = 0;
	bool wingsState = false, hangState = false;
	bool matchLoading = false;
	int matchLoadingTimer = int(millis());
	std::string intake_state = "OFF";
	frontLeftWing.set_value(LOW);
	int cataPos = 15;
	while (true) {

		x1 = controls.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		y1 = controls.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		leftMotors = y1 + x1;
		rightMotors = y1 - x1;

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
			cataPos += 180;
		}

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			cataPos += 60;
		}
		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
			cataPos -= 60;
		}

		if (controls.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			intake = 127;
		}
		else if (controls.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake = -127;
		}
		else {
			intake = 0;
		}

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
			wingsState = !wingsState;
			frontLeftWing.set_value(wingsState);
			frontRightWing.set_value(wingsState);
		}

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			hangState = !hangState;
			sideHang.set_value(hangState);
		}

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			matchLoading = !matchLoading;
		}

		if (matchLoading && distance_sensor.get() <= 10 && matchLoadingTimer <= int(millis())) {
			cataPos += 180;
			matchLoadingTimer = int(millis()) + 220;
			lcd::set_text(line++, "LAUNCHED");
		}

		lcd::set_text(line++, "distance:" + std::to_string(distance_sensor.get()));
		lcd::set_text(line++, "line test");

		line = 0;
		delay(20);
	}
}