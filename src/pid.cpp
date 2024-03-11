#include "main.h"
#include "declerations.hpp"
#include "pid.hpp"

double getMotorGroupPos() {
	double leftSum = 0;
	for (double num : leftMotors.get_positions()) {
		leftSum += num;
	}
	leftSum /= double(leftMotors.get_positions().size());

	double rightSum = 0;
	for (double num : rightMotors.get_positions()) {
		rightSum += num;
	}
	rightSum /= double(rightMotors.get_positions().size());

	double totalAverage = (leftSum + rightSum) / 2.0;
	return totalAverage * (0.6 / 360.0 * 3.25 * 3.14159265358979323846);
}

float reduceTo(float num, float lower, float upper) {
	float diff = upper - lower;
	while (!(num >= lower && num < upper)) {
		if (num < lower) {
			num += diff;
		}
		if (num >= upper) {
			num -= diff;
		}
	}
	return(num);
}

float clamp(float input, float min, float max) {
	if (input > max) { return(max); }
	if (input < min) { return(min); }
	return(input);
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

	PID(float error, float kp, float ki, float kd, float starti) :
		error(error),
		kp(kp),
		ki(ki),
		kd(kd),
		starti(starti) {
	}

	PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout) :
		error(error),
		kp(kp),
		ki(ki),
		kd(kd),
		starti(starti),
		settle_error(settle_error),
		settle_time(settle_time),
		timeout(timeout) {
	}

	float compute(float error) {
		if (fabs(error) < starti) {
			accumulated_error += error;
		}
		if ((error > 0 && 0 < previous_error) || (error < 0 && 0 > previous_error)) {
			accumulated_error = 0;
		}
		output = kp * error + ki * accumulated_error + kd * (error - previous_error);

		previous_error = error;


		if (fabs(error) < settle_error) {
			time_spent_settled += 20;
		}
		else {
			time_spent_settled = 0;
		}

		time_spent_running += 20;

		return output;
	}

	bool is_settled() {
		if (time_spent_running > timeout || time_spent_settled > settle_time) {
			return true;
		}
		return false;
	}
};

float get_absolute_heading() {
	return(reduceTo(gyro.get_heading(), 0, 360));
}

void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd) {
	float desired_heading = angle;
	PID turnPID(reduceTo(angle - get_absolute_heading(), -180, 180), turn_kp, turn_ki, turn_kd, 0, turn_settle_error, turn_settle_time, turn_timeout);
	while (turnPID.is_settled() == false) {
		float error = reduceTo(angle - get_absolute_heading(), -180, 180);
		float output = turnPID.compute(error);
		output = clamp(output, -turn_max_voltage, turn_max_voltage);
		leftMotors = output * 127.0 / 100.0;
		rightMotors = -output * 127.0 / 100.0;
		pros::delay(20);
	}
}

void drive_distance(float distance, float drive_max_voltage, float drive_settle_error, float settle_time, float timeout, float drive_kp, float drive_ki, float drive_kd) {

	PID drivePID(distance, drive_kp, drive_ki, drive_kd, 0, drive_settle_error, settle_time, timeout);
	float start_average_position = getMotorGroupPos();
	float average_position = start_average_position;
	while (drivePID.is_settled() == false) {
		average_position = getMotorGroupPos();
		float drive_error = distance + start_average_position - average_position;
		float drive_output = drivePID.compute(drive_error);

		drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);

		leftMotors = drive_output;
		rightMotors = drive_output;
		pros::delay(20);
	}
}
