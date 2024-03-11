#include "main.h"
#include "declerations.hpp"
#include "pid.hpp"

void maybeAWP() {
	intake.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	gyro.set_heading(135);
	drive_distance(-12);
	turn_to_angle(180);
	drive_distance(-24);
	drive_distance(24);
	turn_to_angle(135);
	backWings.set_value(HIGH);
	drive_distance(12);
	turn_to_angle(90);
	drive_distance(32);
	backWings.set_value(LOW);
}