  #include "Tower.h"

  void Tower::Free(){//release back latch
    SUPPOSED_TO_BE_A_PISTON.rotateTo(HYPOTHETICAL_PISTON_IN, rotationUnits::deg, 127, velocityUnits::pct,false);
  }
  void Tower::Lock(){//drop latch
    SUPPOSED_TO_BE_A_PISTON.rotateTo(HYPOTHETICAL_PISTON_OUT, rotationUnits::deg, 127, velocityUnits::pct,false);
  }
  //move the tower at a certain speed
  void Tower::spin(int speed){
    //Spin both motors at the same speed
    //no need to invert directions as they were inverted when instantiating motors
    LeftMotor.spin(directionType::fwd,speed, percentUnits::pct);
    RightMotor.spin(directionType::fwd,speed, percentUnits::pct);
  }
  //Move the tower to a preset height
  void Tower::MoveTo(){

  }
  //Returns the current rotation of the Tower
  int Tower::GetRotation(void){
    //make sure both the values are positive and take the average rotation
    return Config.🚒.angle(rotationUnits::deg);
  }
  void Tower::stop(){
    //No need to write a PID to hold the tower in place, the vex one is okay(they dont show source code which scares me)
    LeftMotor.stop(brakeType::hold);
    RightMotor.stop(brakeType::hold);
  }