#include "main.h"
#include "declerations.hpp"

float cataPos = 15;
bool matchLoading = false;
int matchLoadingTimer = int(pros::millis());

void intakeControl() {
	if (controls.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		intake = 127;
	}
	else if (controls.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		intake = -127;
	}
	else {
		intake = 0;
	}
}

void catapultControl() {
	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
		cataPos += 180;
	}
	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		cataPos += 60;
	}
	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		cataPos -= 60;
	}

	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		matchLoading = !matchLoading;
	}
	if (matchLoading && distance_sensor.get() <= 10 && matchLoadingTimer <= int(pros::millis())) {
		cataPos += 180;
		matchLoadingTimer = int(pros::millis()) + 220;
	}

	catapult.move_absolute(cataPos, 127);
}

bool frontWingsState = false, backWingsState = false, hangState = false;
void frontWingsControl() {
	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
		frontWingsState = !frontWingsState;
		frontLeftWing.set_value(frontWingsState);
		frontRightWing.set_value(frontWingsState);
	}
}
void backWingsControl() {
	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
		frontWingsState = !frontWingsState;
		frontLeftWing.set_value(frontWingsState);
		frontRightWing.set_value(frontWingsState);
	}
}
void hangControl() {
	if (controls.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		hangState = !hangState;
		sideHang.set_value(hangState);
	}
}