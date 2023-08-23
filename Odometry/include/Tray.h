#pragma once
#include "vex.h"
class Tray{
  private:
  float stackMax = 0;
  public:
  bool isPIDRunning = false;
  motor Stacker = motor(PORT15,gearSetting::ratio36_1,true);
  Tray(float);
  void iterateStack(float kP,float kI, float kD);
  void stack(int timeout,float kP,float kI,float kD);
  void retract(float speed);
  float value();
  void reset();
  void spin(int speed);
  void stop();

  bool isRunning();
};