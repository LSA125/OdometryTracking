#include "PID.h"
PID::PID(float kP,float kI,float kD)
{
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
}
PID::PID(float kP,float kI,float kD,float ILim)
{
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->ILim = ILim;
}
float const PID::GetValue(float err){
  D = (err-P);
  I = ((I + err) * kI > ILim ? I : I + err);
  P = err;
  return((P * kP)+(I * kI)+(D*kD));
}
void PID::printFirstTime(unsigned int in){
  if(time == 0){
    time = in;
    Config.Controller.Screen.clearLine(2);
    Config.Controller.Screen.setCursor(2,0);
    Config.Controller.Screen.print("time: %d",time);
  }
}