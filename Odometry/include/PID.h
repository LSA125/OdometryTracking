#pragma once
#include "vex.h"
class PID{
  private:

  float P=0,D=0,I=0;
  float kP = 0,kD = 0,kI = 0,ILim = 30;
  unsigned int time = 0;
  public:

  PID(float kP,float kI,float kD);
  PID(float kP,float kI,float kD,float ILim);
  float const GetValue(float err);
  void printFirstTime(unsigned int in);
};