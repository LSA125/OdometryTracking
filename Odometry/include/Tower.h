#pragma once
#include "vex.h"
class Tower//TOWER MAX: 2140
{
  private:
  motor RightMotor = motor(PORT1,gearSetting::ratio18_1,true);
  motor LeftMotor = motor(PORT2,gearSetting::ratio18_1,false);
  const float HYPOTHETICAL_PISTON_OUT = 420,HYPOTHETICAL_PISTON_IN = 180;
  

  
  public:
  array<int,5>Preset =
  {
    80,  //base rotation
    100, //basesaefasf
    127,  //rotation for low branches on MOGOS
    169,  //rotation for high branches on MOGOS
    //145 for enter 138 to pull out
  };
  motor SUPPOSED_TO_BE_A_PISTON = motor(PORT4,gearSetting::ratio18_1,false);
  unsigned int preset_selector = 0;
  //Ball array, {Bottom,Middle,Top}
  void Free();//release back latch
  void Lock();//drop latch
  void spin(int speed);//move at constant speed
  void MoveTo();//move tower to a certain height
  int GetRotation(void);//returns 
  void stop();
};