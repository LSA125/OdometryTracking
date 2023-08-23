#include "Intake.h"

void Intake::spin(int speed)
{
  NeedleIntake.spin(directionType::fwd,speed,velocityUnits::pct);
}
void Intake::rotateFor(unsigned int time,int speed)
{
  spin(speed);
  task::sleep(time);
  stop();
}

void Intake::stop()
{
  NeedleIntake.stop(brakeType::brake);
}