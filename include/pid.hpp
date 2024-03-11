#ifndef _PID_
#define _PID_


extern void turn_to_angle(float angle, float turn_max_voltage = 127, float turn_settle_error = 10, float turn_settle_time = 300, float turn_timeout = 1000, float turn_kp = 1.2, float turn_ki = 6, float turn_kd = 5);

extern void drive_distance(float distance, float drive_max_voltage = 127, float drive_settle_error = 1.5, float settle_time = 300, float timeout = 1000, float drive_kp = 20, float drive_ki = 2, float drive_kd = 3);

#endif