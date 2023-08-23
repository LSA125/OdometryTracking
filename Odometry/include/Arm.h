
#pragma once
#include "vex.h"
class Arm
{
  private:
  int preset = 0;
  static const short presetSize = 4;
  float armPresets[presetSize] = {0,0,0,0};
  void moveArm();
  int isValid(int in);
  public:
  bool disablePreset = true;
  motor armMotor = motor(PORT1,gearSetting::ratio36_1,true);

  Arm(float[presetSize-1]);

  void armUp();
  void armDown();

  void operator++();
  void operator++(int);
  void operator--();
  void operator--(int);
  void operator+=(int);
  void operator-=(int);
  void operator=(int);

  float value();
  void reset();
  void spin(int);
  void stop();
};