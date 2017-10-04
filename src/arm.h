#ifndef arm_h
#define arm_h

double armInput, armOutput, armSetpoint;
double Kp_arm = 1, Ki_arm = 0, Kd_arm = 0;
PID armPID(&armInput,&armOutput,&armSetpoint,Kp_arm,Ki_arm,Kd_arm,false);

#endif
