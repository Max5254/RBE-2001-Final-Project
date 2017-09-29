/*

*/

#ifndef PID_h
#define PID_h


class PID
{
public:
  PID(double*,double*,double*,double,double,double,bool);
  void Enable(bool);
  void Compute();
  void SetOutputLimits(double,double);
  void SetControllerDirection(bool);

  void SetTunings(double,double,double);

  double GetKp();						  // These functions query the pid for interal values.
  double GetKi();						  //  they were created mainly for the pid front-end,
  double GetKd();						  // where it's important to know what is actually
  bool GetEnabled();						  //  inside the PID.
  bool GetDirection();

private:
  void Initialize();

  double myInput;              // * Pointers to the Input, Output, and Setpoint variables
  double myOutput;             //   This creates a hard link between the variables and the
  double mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.

  double Kp;                  // * (P)roportional Tuning Parameter
  double Ki;                  // * (I)ntegral Tuning Parameter
  double Kd;                  // * (D)erivative Tuning Parameter

  double  errorIntegral, lastInput;

  bool controllerDirection;
  bool pOn;


  double outMin, outMax;
};

#endif
