
#pragma once

#include "vex.h"

class Sensors
{
  private:
  //these are all fixes glitches in the vex gyro software
  void waitUntilCalibrated();
  public:
  float angleWrap(float);
  float angleWrapRad(float);
  float ToDegrees(float);
  float ToRadians(float);
  //get the gyro value with fix and value reset implimented
  float getGyro();
  //calibrate the gyro
  void calibrateGyro();
  //set gyro value
  void setGyro(float);

};
