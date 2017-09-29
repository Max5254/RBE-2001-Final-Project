 #include "PID.h"

 PID::PID(double* Input, double* Output, double* Setpoint,double Kp, double Ki, double Kd, bool ControllerDirection)
{
  myOutput = *Output;
  myInput = *Input;
  mySetpoint = *Setpoint;
  pOn = false;

  PID::SetOutputLimits(-1, 1);				//default output limit corresponds to
                                      //the arduino pwm limits

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd);

}

void PID::Enable(bool enable){
  pOn = enable;
}

void PID::Compute()
{
   if(!pOn)
    return;

      /*Compute all the working error variables*/
      double input = myInput;
      double error = mySetpoint - input;
      double errorDerivative = (input - lastInput);
      errorIntegral+= error;

	   double output = (Kp * error) + (Ki * errorIntegral) + (Kd * errorDerivative) ;

	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;

      myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
}

void PID::SetOutputLimits(double min, double max){
  outMin = min;
  outMax = max;
}

void PID::SetControllerDirection(bool forwards){
  controllerDirection = forwards;
}

void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

  this->Kp=Kp;
  this->Ki=Ki;
  this->Kd=Kd;
}


double PID::GetKp(){ return  Kp; }
double PID::GetKi(){ return  Ki;}
double PID::GetKd(){ return  Kd;}
bool PID::GetDirection(){ return controllerDirection;}
bool PID::GetEnabled(){ return pOn;}
