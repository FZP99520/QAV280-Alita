#ifndef __AHRS_H
#define __AHRS_H

#define kp_acc_fast 2.0f
#define ki_acc_fast 0.02f
#define kp_acc_slow 0.2f
#define ki_acc_slow 0.00025f
#define kp_yaw_fast 0.05f
#define ki_yaw_fast 0.02f
#define kp_yaw_slow 0.1f;
#define ki_yaw_slow 0.00f
extern float q0,q1,q2,q3;

void MahonyAHRSupdateIMU(float gx, float gy, float gz,float ax,float ay,float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif
