#ifndef _DECLARE_
#define _DECLARE_
#include "main.h"

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


#endif