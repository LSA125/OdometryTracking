
#include "Arm.h"

void Arm::moveArm(){
  if(!disablePreset){
    armMotor.rotateTo(armPresets[preset],rotationUnits::deg,127,velocityUnits::pct,false);
  }
}
int Arm::isValid(int in){
  if(in < 0){
    return 0;
  }else if(in >= presetSize){
    return presetSize-1;
  }
  return in;
}

Arm::Arm(float presets[presetSize-1]){
  for(int x = 0;x < presetSize-1;++x){
    armPresets[x+1] = presets[x];
  }
}

void Arm::armUp(){
  preset = isValid(++preset);
  moveArm();
}
void Arm::armDown(){
  preset = isValid(--preset);
  moveArm();
}

void Arm::operator++(){
  preset = isValid(++preset);
  moveArm();
}
void Arm::operator--(){
  preset = isValid(--preset);
  moveArm();
}
void Arm::operator+=(int a){
  preset = isValid(preset + a);
  moveArm();
}
void Arm::operator-=(int a){
  preset = isValid(preset - a);
  moveArm();
}
void Arm::operator=(int a){
  preset = isValid(a);
  moveArm();
}

float Arm::value(){
  return armMotor.rotation(rotationUnits::deg);
}
void Arm::reset(){
  armMotor.resetRotation();
}
void Arm::spin(int in){
  // test if presets are finished running and if the arm isnt running down passed the switch
  if(!(in < 0 && Config.LimitArm.pressing())){
    armMotor.spin(directionType::fwd,in,velocityUnits::pct);
  }
  if(Config.LimitArm.pressing()){
    reset();
  }
}
void Arm::stop(){
  if(armMotor.isDone()){
    armMotor.stop(brakeType::hold);
  }
}