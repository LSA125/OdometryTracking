#include "Sensors.h"

//waits until the gyro is calibrated
void Sensors::waitUntilCalibrated()
{
  while(Config.Inertial.isCalibrating()){
    task::sleep(10);
  }
}
void Sensors::calibrateGyro(){
  Config.Inertial.calibrate();
}
//turns gyro values like -10 into 350 or 370 into 10
float Sensors::angleWrap(float gyroVal){
  //too low
  while(gyroVal < -180){
    gyroVal += 360;
  }
  //too high
  while(gyroVal > 180){
    gyroVal -= 360;
  }
  return gyroVal;
}
float Sensors::angleWrapRad(float gyroVal){
  //too low
  while(gyroVal < -1*PI){
    gyroVal += 2*PI;
  }
  //too high
  while(gyroVal > PI){
    gyroVal -= 2*PI;
  }
  return gyroVal;
}
float Sensors::ToDegrees(float angle){
  return angle * (180/PI);
}
float Sensors::ToRadians(float angle){
  return angle * (PI/180);
}
//get the gyro value with fix and value reset implimented
float Sensors::getGyro()
{
  //wait until the gyro is calibrated
  waitUntilCalibrated();
  //retun fixed gyro value of the Real gyro value + the manipulator
  return angleWrap(Config.Inertial.rotation(rotationUnits::deg));
}

//set gyro value
void Sensors::setGyro(float gyroVal)
{
  Config.Inertial.setRotation(angleWrap(gyroVal), rotationUnits::deg);
}