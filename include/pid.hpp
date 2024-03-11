#ifndef _PID_
#define _PID_

extern void turn_to_angle(float angle, float turn_max_voltage = default_turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd);
extern void drive_distance(float distance, float drive_kp = default_drive_kp, float drive_ki = default_drive_ki, float drive_kd = default_drive_kd, float drive_max_voltage = default_drive_max_voltage, float drive_settle_error = default_drive_settle_error, float settle_time = default_drive_settle_time, float timeout = default_drive_timeout);

#endif