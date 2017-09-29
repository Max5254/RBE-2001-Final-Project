#ifndef drive_h
#define drive_h


double driveInput, driveOutput, driveSetpoint;
double Kp_drive = 1, Ki_drive = 0, Kd_drive = 0;
PID drivePID(&driveInput,&driveOutput,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,false);

double straightInput, straightOutput, straightSetpoint;
double Kp_straight = 1, Ki_straight = 0, Kd_straight = 0;
PID straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,false);

double turnInput, turnOutput, turnSetpoint;
double Kp_turn = 1, Ki_turn = 0, Kd_turn = 0;
PID turnPID(&turnInput,&turnOutput,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,false);

void turnToAngle(double);
void driveToPoint(double,double,double);
void driveToLineAtPoint(double,double);
void driveToButton();

#endif
