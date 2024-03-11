#ifndef _DECLARE_
#define _DECLARE_
#include "main.h"

extern pros::Controller controls;

extern pros::Motor frontLeft;
extern pros::Motor midLeft;
extern pros::Motor backLeft;
extern pros::Motor_Group leftMotors;

extern pros::Motor frontRight;
extern pros::Motor midRight;
extern pros::Motor backRight;
extern pros::Motor_Group rightMotors;

extern pros::Motor intake1;
extern pros::Motor intake2;
extern pros::Motor_Group intake;
extern pros::Motor catapult;

extern pros::Imu gyro;
extern pros::Distance distance_sensor;

extern pros::ADIDigitalOut frontLeftWing;
extern pros::ADIDigitalOut frontRightWing;
extern pros::ADIDigitalOut sideHang;
extern pros::ADIDigitalOut backWings;


#endif