#pragma once
#include "vex.h"

class Intake
{
  private:
  
  public:
  motor NeedleIntake = motor(PORT6,gearSetting::ratio18_1,true);
  void spin(int);
  void rotateFor(unsigned int,int);
  void stop();
};