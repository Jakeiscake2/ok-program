#include "main.h"
#include "declerations.hpp"

pros::Controller controls(pros::E_CONTROLLER_MASTER);

pros::Motor frontLeft(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor midLeft(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backLeft(3, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftMotors({ frontLeft,midLeft,backLeft });

pros::Motor frontRight(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor midRight(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor backRight(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group rightMotors({ frontRight,midRight,backRight });

pros::Motor intake1(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake2(8, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group intake({ intake1,intake2 });
pros::Motor catapult(9, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Imu gyro(19);
pros::Distance distance_sensor(20);

pros::ADIDigitalOut frontLeftWing('A');
pros::ADIDigitalOut frontRightWing('B');
pros::ADIDigitalOut sideHang('C');
pros::ADIDigitalOut backWings('D');