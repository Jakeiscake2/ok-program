#include "main.h"
using namespace pros;

pros::Controller controls(E_CONTROLLER_MASTER);

pros::Motor frontLeft(1, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
pros::Motor midLeft(2, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
pros::Motor backLeft(3, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftMotors({ frontLeft,midLeft,backLeft });

pros::Motor frontRight(4, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
pros::Motor midRight(5, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
pros::Motor backRight(6, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group rightMotors({ frontRight,midRight,backRight });

pros::Motor intake1(7, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
pros::Motor intake2(8, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group intake({ intake1,intake2 });
pros::Motor catapult(9, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);

pros::Imu gyro(19);
Distance distance_sensor(20);

pros::ADIDigitalOut frontLeftWing('A');
pros::ADIDigitalOut frontRightWing('B');
pros::ADIDigitalOut sideHang('C');
pros::ADIDigitalOut backWings('D');

double calcAverage(std::vector<double> input) {
	double sum = 0;
	for (double num : input) {
		sum += num;
	}
	return sum / double(input.size());
}

#define M_PI		3.14159265358979323846
float drive_in_to_deg_ratio = (0.6 / 360.0 * M_PI * 3.25);

float reduce_0_to_360(float angle) {
	while (!(angle >= 0 && angle < 360)) {
		if (angle < 0) { angle += 360; }
		if (angle >= 360) { angle -= 360; }
	}
	return(angle);
}

float reduce_negative_180_to_180(float angle) {
	while (!(angle >= -180 && angle < 180)) {
		if (angle < -180) { angle += 360; }
		if (angle >= 180) { angle -= 360; }
	}
	return(angle);
}

float reduce_negative_90_to_90(float angle) {
	while (!(angle >= -90 && angle < 90)) {
		if (angle < -90) { angle += 180; }
		if (angle >= 90) { angle -= 180; }
	}
	return(angle);
}

float to_rad(float angle_deg) {
	return(angle_deg / (180.0 / M_PI));
}

float to_deg(float angle_rad) {
	return(angle_rad * (180.0 / M_PI));
}

float clamp(float input, float min, float max) {
	if (input > max) { return(max); }
	if (input < min) { return(min); }
	return(input);
}

float to_volt(float percent) {
	return(percent * 127.0 / 100.0);
}

class PID {
public:
	float error = 0;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float starti = 0;
	float settle_error = 0;
	float settle_time = 0;
	float timeout = 0;
	float accumulated_error = 0;
	float previous_error = 0;
	float output = 0;
	float time_spent_settled = 0;
	float time_spent_running = 0;

	PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);

	PID(float error, float kp, float ki, float kd, float starti);

	float compute(float error);

	bool is_settled();
};


PID::PID(float error, float kp, float ki, float kd, float starti) :
	error(error),
	kp(kp),
	ki(ki),
	kd(kd),
	starti(starti) {
};

PID::PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout) :
	error(error),
	kp(kp),
	ki(ki),
	kd(kd),
	starti(starti),
	settle_error(settle_error),
	settle_time(settle_time),
	timeout(timeout) {
};


float PID::compute(float error) {
	if (fabs(error) < starti) {
		accumulated_error += error;
	}
	if ((error > 0 && previous_error < 0) || (error < 0 && previous_error>0)) {
		accumulated_error = 0;
	}
	output = kp * error + ki * accumulated_error + kd * (error - previous_error);

	previous_error = error;

	if (fabs(error) < settle_error) {
		time_spent_settled += 10;
	}
	else {
		time_spent_settled = 0;
	}

	time_spent_running += 10;

	return output;
}

bool PID::is_settled() {
	if (time_spent_running > timeout && timeout != 0) {
		return(true);
	}
	if (time_spent_settled > settle_time) {
		return(true);
	}
	return(false);
}
float get_absolute_heading() {
	return(reduce_0_to_360(gyro.get_heading()));
}

float get_left_position_in() {
	return(calcAverage(leftMotors.get_positions()) * drive_in_to_deg_ratio);
}

float get_right_position_in() {
	return(calcAverage(rightMotors.get_positions()) * drive_in_to_deg_ratio);
}

float turn_starti = 0;
float drive_starti = 0;
float heading_starti = 0;
float desired_heading = 0;

void turn_to_angle1(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti) {
	float desired_heading = angle;
	PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
	while (turnPID.is_settled() == false) {
		float error = reduce_negative_180_to_180(angle - get_absolute_heading());
		float output = turnPID.compute(error);
		output = clamp(output, -turn_max_voltage, turn_max_voltage);
		leftMotors = to_volt(output);
		rightMotors = to_volt(-output);
		delay(20);
	}
	//PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);
}

/*
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

  chassis.set_turn_constants(127, 1, 4, 100, 1);

  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);

*/

void drive_distance1(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
	float desired_heading = heading;
	PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
	PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
	float start_average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
	float average_position = start_average_position;
	while (drivePID.is_settled() == false) {
		average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
		float drive_error = distance + start_average_position - average_position;
		float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
		float drive_output = drivePID.compute(drive_error);
		float heading_output = headingPID.compute(heading_error);

		drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
		heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

		leftMotors = drive_output;
		rightMotors = drive_output;
		delay(20);
	}
}

void drive_distance(float distance) {
	drive_distance1(distance, desired_heading, 127, 127, 1.5, 300, 1000, 20, 2, 3, drive_starti, 60, 0.4, 0, heading_starti);
}

void turn_to_angle(float angle) {
	turn_to_angle1(angle, 127, 10, 300, 1000, 1.2, 6, 5, turn_starti);
}


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

void maybeAWP() { //push in then get out
	//back parallel to matchload bar, triball push in
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

void autonomous() {
	intake.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	maybeAWP();
	/*
	drive_distance(5);
	turn_to_angle(45);
	drive_distance(5);
	turn_to_angle(45);
	*/
}

void opcontrol() {
	catapult.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	catapult.tare_position();
	int x1, y1 = 0;
	int line = 0;
	bool wingsState = false, hangState = false;
	bool cataBroken = false;
	bool matchLoading = false;
	int matchLoadingTimer = int(millis());
	std::string intake_state = "OFF";
	frontLeftWing.set_value(LOW);
	int cataPos = 15;
	while (true) {

		/*
				x1 = controls.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
				y1 = controls.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

				x2 = controls.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
				y2 = controls.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

				left_motors = y2 - y1 + x1 + x2;
				right_motors = y2 - y1 - x1 - x2;
		*/

		x1 = controls.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		y1 = controls.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		leftMotors = y1 + x1;
		rightMotors = y1 - x1;

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
			cataPos += 180;
		}

		if (controls.get_digital(E_CONTROLLER_DIGITAL_LEFT)) {
			cataBroken = true;
		}
		if (controls.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
			cataBroken = false;
		}

		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_UP) && cataBroken) {
			catapult.move_relative(30, 127);
		}
		if (controls.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN) && cataBroken) {
			catapult.move_relative(-30, 127);
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