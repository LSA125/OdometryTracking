
#pragma once
#include "Intake.h"
#include "Tower.h"
#include "Sensors.h"
extern Intake intakes;
extern Tower tower;
class Chassis{
  private:
    //expoDrive function: Joystick value, Exponent of Line, dead zone and minimun motor val
    int expoDrive(int joyVal, float driveExp, int joyDead, int motorMin);
    const float ROBOT_WIDTH = 15.048;
    const float ROBOT_HEIGHT = 6.1;
    const float INCHES_PER_DEGREES = 3.25f * PI/ 360;
    struct{
      float x,y,angle;
    }position;

                          /*Pure Pursuit Sub Methods*/
    vector<array<float,3>> LinePoints;
    float signum(float in){
      return in < 0 ? -1 : 1;
    }
    array<float,3> getLookAhead(float OriginX,float OriginY,float LookAheadDistance,unsigned int start,unsigned int end);
    

  public:
    motor LFWheel = motor(PORT19,gearSetting::ratio18_1,false);
    motor LBWheel = motor(PORT11,gearSetting::ratio18_1,false);
    motor RFWheel = motor(PORT12,gearSetting::ratio18_1,true);
    motor RBWheel = motor(PORT20,gearSetting::ratio18_1,true);

    Sensors sensors;
    //getters
    float getX() const;
    float getY() const;
    float getAngle() const;
    //setters
    void setX(float X_Coordinate);
    void setY(float Y_Coordinate);
    void setAngle(float Robot_Angle);
    void setPoints(vector<array<float,3>>);
    void addPoints(vector<array<float,3>>);
    void addPoint(float X_Coordinate, float Y_Coordinate);
    void addPoint(float X_Coordinate, float Y_Coordinate,float angle);
      /*AUTONOMOUS FUNCTIONS*/
    //odometry - tracking position
    void trackPosition();
    //move in direction + % of rotation
    void Move(float degrees,float rotationPower, float speed);
    //pure pursuit algorithm
    void Follow_Path(float Look_Ahead_Distance,unsigned int start,unsigned int end, unsigned int timeout);
    void Follow_Path(float Look_Ahead_Distance,unsigned int start,unsigned int end,unsigned int speed, unsigned int timeout);

    //PID functions towards an absolute position
    //move to point stored in LinePoints
    void PIDMoveTo(float kP,float kI,float kD, unsigned int timeout, unsigned int Increment, float angle);
    void MoveTo(int speed, unsigned int timeout, unsigned int Increment, float angle);
    //turn to point
    void TurnTo(float kP, float kI, float kD, unsigned int timeout, float angle);

    //vision sensor- put ball in center of screen
    void VisionAllign(bool isRed,unsigned int timeout);
      /*USER FUNCTIONS*/
    
    void Arcade(float xAxis,float yAxis);
    void Tank(float yAxisLeft, float yAxisRight);
    void Mechanum(float StrafeAxis,float TurnAxis,float yAxis);

    void stop();
};