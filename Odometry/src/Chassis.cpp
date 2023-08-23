#include "Chassis.h"
Intake intakes;
Tower tower;
  /*PRIVATE FUNCTIONS*/

int Chassis::expoDrive(int joyVal, float driveExp, int joyDead, int motorMin) {
  int joySign;
  //Max value = 127 - accidental movement
  int joyMax = 127 - joyDead;
  int joyLive = abs(joyVal) - joyDead;
  if (joyVal > 0) joySign = 1;
  else if (joyVal < 0) joySign = -1;
  else joySign = 0; 
  //Define power
  int power = joySign * (motorMin + ((127 - motorMin) * pow(joyLive, driveExp) / pow(joyMax, driveExp)));
  return power;
}

  /*AUTONOMOUS FUNCTIONS*/
float Chassis::getX() const{
  return position.x;
}
float Chassis::getY() const{
  return position.y;
}
float Chassis::getAngle() const{
  return position.angle;
}
void Chassis::setX(float in){
  position.x = in;
}
void Chassis::setY(float in) {
  position.y = in;
}
void Chassis::setAngle(float in){
  position.angle = sensors.ToRadians(in);
  sensors.setGyro(in);
}
void Chassis::setPoints(vector<array<float,3>> in){
  LinePoints = in;
}
void Chassis::addPoints(vector<array<float,3>> in){
  LinePoints.insert(LinePoints.end(),in.begin(),in.end());
}
void Chassis::addPoint(float x, float y){
  array<float,3> in = {x,y};
  LinePoints.push_back(in);
}
void Chassis::addPoint(float x, float y,float angle){
  array<float,3> in = {x,y,angle};
  LinePoints.push_back(in);
}
void Chassis::Follow_Path(float LookAheadDistance,unsigned int start,unsigned int end,unsigned int timeout){//TODO
  array<float,3> lookahead;
  float DistanceToTarget = LookAheadDistance;
  PID SpeedControl(180,0,0);
  Config.Brain.Timer.reset();
  while(fabs(DistanceToTarget) > 1 && Config.Brain.Timer.time(timeUnits::msec) < timeout){
    lookahead = getLookAhead(position.x, position.y, LookAheadDistance,start,end);
    lookahead[2] = sensors.ToRadians(lookahead[2]);
    bool IsValid = lookahead[1] - position.y == 0 ? false : true;
    float  TargetDirection = !IsValid ? 0 : atan2(lookahead[0]-position.x,lookahead[1] - position.y);
    //a^2 + b^2 = c^2
    float  DistanceToTarget = !IsValid ?LinePoints[end][1] - position.y : sqrt((LinePoints[end][1] - position.y) * (LinePoints[end][1] - position.y) + (LinePoints[end][0]-position.x) * (LinePoints[end][0]-position.x));

    float angleN = fabs(sensors.angleWrapRad(lookahead[2]-position.angle))*0.5f,distanceN = fabs(DistanceToTarget/40);
    float angleSpeed;//variable to represent what % of motion should be angular
    angleSpeed = (angleN/distanceN);
    if(angleSpeed > 1)//make sure no weird values get through
      angleSpeed = 1;
    if(angleSpeed < 0)
      angleSpeed = 0;
    if(sensors.angleWrapRad(lookahead[2]-position.angle) < 0){//if the robot should turn left
      angleSpeed *= -1;
    }
    Move(sensors.ToDegrees(TargetDirection-position.angle),angleSpeed,SpeedControl.GetValue(angleN + distanceN));

    task::sleep(10);
  }
  stop();
}
//Pure pursuit algorithm without running a PID
void Chassis::Follow_Path(float LookAheadDistance,unsigned int start,unsigned int end,unsigned int speed,unsigned int timeout){//TODO
  array<float,3> lookahead;//coordinates of the point we're heading to
  float DistanceToTarget = LookAheadDistance;
  Config.Brain.Timer.reset();
  while(fabs(DistanceToTarget) > 1 && Config.Brain.Timer.time(timeUnits::msec) < timeout){
    lookahead = getLookAhead(position.x, position.y, LookAheadDistance,start,end);
    lookahead[2] = sensors.ToRadians(lookahead[2]);
    bool IsValid = lookahead[1] - position.y == 0 ? false : true;
    float  TargetDirection = !IsValid ? 0 : atan2(lookahead[0]-position.x,lookahead[1] - position.y);
    //if the point is PERFECTLY straight ahead, distance is y target point - y position. Otherwise a^2 + b^2 = c^2
    float  DistanceToTarget = !IsValid ?LinePoints[end][1] - position.y : sqrt((LinePoints[end][1] - position.y) * (LinePoints[end][1] - position.y) + (LinePoints[end][0]-position.x) * (LinePoints[end][0]-position.x));

    float angleN = fabs(sensors.angleWrapRad(lookahead[2]-position.angle))*0.5f,distanceN = fabs(DistanceToTarget/40);
    float angleSpeed;//variable to represent what % of motion should be angular
    angleSpeed = (angleN/distanceN);
    if(angleSpeed > 1)//make sure no weird values get through
      angleSpeed = 1;
    if(angleSpeed < 0)
      angleSpeed = 0;
    if(sensors.angleWrapRad(lookahead[2]-position.angle) < 0){//if the robot should turn left
      angleSpeed *= -1;
    }
    Move(sensors.ToDegrees(TargetDirection-position.angle),angleSpeed,speed);

    task::sleep(10);
  }
  stop();
}
void Chassis::trackPosition(){
  //last encoder values-- resetting encoder values on each cycle would interfere with other functions
  static float LastL = 0,LastR = 0,LastB = 0;
  //angle is the change in rotation
  float angle = 0;
  //convert from encoder values to inches traveled
  float LeftDistance = ((Config.LeftEncoder.position(rotationUnits::deg)-LastL))*INCHES_PER_DEGREES,
  RightDistance = ((Config.RightEncoder.position(rotationUnits::deg)-LastR))*INCHES_PER_DEGREES,
  BackDistance = ((Config.BackEncoder.position(rotationUnits::deg)-LastB))*INCHES_PER_DEGREES;

  if(LeftDistance == RightDistance){
    float avgOrientation = (position.angle + angle)*0.5f;
    float hypot = sqrt(BackDistance * BackDistance + RightDistance * RightDistance);
    float newAngle = atan2(RightDistance,BackDistance);
    position.x += hypot * cos(newAngle - avgOrientation);
    position.y += hypot * sin(newAngle - avgOrientation);
  }else{
    //get the direction, use the inertial sensor until the program has run for 2 minutes, then use encoders after
    //inertial sensors are accurate but drift builds up over time, encoder turns are reliably less accurate
    angle = (LeftDistance - RightDistance)/ROBOT_WIDTH;//sensors.ToRadians(sensors.getGyro()) - position.angle;// : 
    float deltaX = 2*(BackDistance/angle+ROBOT_HEIGHT) * sin(angle*0.5f);
    float deltaY = 2*(RightDistance/angle+ROBOT_WIDTH*0.5f) * sin(angle*0.5f);
    float avgOrientation = position.angle + angle*0.5f;

    float hypot = sqrt(deltaX * deltaX + deltaY * deltaY);
    float newAngle = atan2(deltaY,deltaX);
    //adjust for offset of avgOrientation
    //separate distance into x distance and y distance

    position.x += hypot * cos(newAngle - avgOrientation);
    position.y += hypot * sin(newAngle - avgOrientation);

    //add the angle travelled to the total angle
    position.angle += angle;
  }
  LastL = Config.LeftEncoder.position(rotationUnits::deg);
  LastR = Config.RightEncoder.position(rotationUnits::deg);
  LastB = Config.BackEncoder.position(rotationUnits::deg);
  task::sleep(2);
}
void Chassis::Move(float degrees, float rotationPower, float speed){
  
  while(degrees < 0){
    degrees+=360;
  }
  //ask me for proof; Equations to get ratios for desired direction
  float power_A = -1*((cos(3*PI*0.25f + sensors.ToRadians(degrees))*(-1*fabs(rotationPower)+1))/(cos(PI*0.25f))),
  power_B = ((cos(PI*0.25f + sensors.ToRadians(degrees))*(-1*fabs(rotationPower)+1))/(cos(PI*0.25f)));

  //assign raw speed to each motor
  float LFSpeed = power_B + rotationPower,
  RFSpeed = power_A - rotationPower,
  LBSpeed = power_A + rotationPower,
  RBSpeed = power_B - rotationPower,
  largest = LFSpeed;


  //set largest to the largest value
  if(fabs(largest) < fabs(RFSpeed)){
    largest = RFSpeed;
  }
  if(fabs(largest) < fabs(LBSpeed)){
    largest = LBSpeed;
  }
  if(fabs(largest) < fabs(RBSpeed)){
    largest = RBSpeed;
  }
  //get speed ratio
  largest /= speed;
  largest = fabs(largest);
  LFWheel.spin(directionType::fwd, LFSpeed/largest, percentUnits::pct);
  RFWheel.spin(directionType::fwd, RFSpeed/largest, percentUnits::pct);
  LBWheel.spin(directionType::fwd, LBSpeed/largest, percentUnits::pct);
  RBWheel.spin(directionType::fwd, RBSpeed/largest, percentUnits::pct);
}
array<float, 3> Chassis::getLookAhead(float x, float y, float r, unsigned int start, unsigned int end) {//TODO
    array<float, 3> lookahead = { -1,0,0 };

    // iterate through all pairs of points
    for (unsigned int i = start; i < end; i++) {
        // form a segment from each two adjacent points
        array<float, 2> segmentStart = { LinePoints[i][0],LinePoints[i][1] };
        array<float, 2> segmentEnd = { LinePoints[i + 1][0],LinePoints[i + 1][1] };

        // translate the segment to the origin
        array<float, 2> p1 = { segmentStart[0] - x, segmentStart[1] - y };
        array<float, 2> p2 = { segmentEnd[0] - x, segmentEnd[1] - y };
        // calculate an intersection of a segment and a circle with radius r (lookahead) and origin (0, 0)
        float dx = p2[0] - p1[0];
        float dy = p2[1] - p1[1];
        float d = sqrt(dx * dx + dy * dy);
        float D = p1[0] * p2[1] - p2[0] * p1[1];

        // if the discriminant is zero or the points are equal, there is no intersection
        float discriminant = r * r * d * d - D * D;
        if (discriminant < 0 || (p1[0] == p2[0] && p1[1] == p2[1])) continue;

        // the x components of the intersecting points
        float x1 = (float)(D * dy + signum(dy) * dx * sqrt(discriminant)) / (d * d);
        float x2 = (float)(D * dy - signum(dy) * dx * sqrt(discriminant)) / (d * d);

        // the y components of the intersecting points
        float y1 = (float)(-D * dx + fabs(dy) * sqrt(discriminant)) / (d * d);
        float y2 = (float)(-D * dx - fabs(dy) * sqrt(discriminant)) / (d * d);

        // whether each of the intersections are within the segment (and not the entire line)
        bool validIntersection1 = (min(p1[0], p2[0]) < x1 && x1 < max(p1[0], p2[0]))
            || (min(p1[1], p2[1]) < y1 && y1 < max(p1[1], p2[1]));
        bool validIntersection2 = (min(p1[0], p2[0]) < x2 && x2 < max(p1[0], p2[0]))
            || (min(p1[1], p2[1]) < y2 && y2 < max(p1[1], p2[1]));

        // select the first one if it's valid
        if (validIntersection1) {
            lookahead = { x1 + x, y1 + y,LinePoints[i+1][2] };
        }

        // select the second one if it's valid and either lookahead is none,
        // or it's closer to the end of the segment than the first intersection
        if (validIntersection2) {
            if ((lookahead[0] == 0 && lookahead[1] == 0) || fabs(x1 - p2[0]) > fabs(x2 - p2[0]) || fabs(y1 - p2[1]) > fabs(y2 - p2[1])) {
                lookahead = { x2 + x, y2 + y,LinePoints[i+1][2] };
            }
        }
    }

    // special case for the very last point on the path
    if (LinePoints.size() > 0) {
        array<float, 2> lastPoint = { LinePoints[end][0],LinePoints[end][1] };

        array<float, 3> endP = { lastPoint[0],lastPoint[1],LinePoints[end][2] };

        // if we are closer than lookahead distance to the end, set it as the lookahead
        if (sqrt((endP[0] - x) * (endP[0] - x) + (endP[1] - y) * (endP[1] - y)) <= r) {
            return endP;
        }
    }
    if (lookahead[0] == -1) {
        float smallest = 500;
        unsigned int increment = 0;
        for (unsigned int i = 0; i < LinePoints.size(); ++i) {
            float DistanceToTarget = sqrt((LinePoints[i][0] - position.x) * (LinePoints[i][0] - position.x) + (LinePoints[i][1] - position.y) * (LinePoints[i][1] - position.y));
            if (smallest > DistanceToTarget)
            {
                increment = i;
                smallest = DistanceToTarget;
            }
        }
        return LinePoints[increment];
    }
    return lookahead;
}
void Chassis::PIDMoveTo(float kP,float kI,float kD, unsigned int timeout, unsigned int Increment, float angle){
  const int precision = 10;
  
  PID SpeedControl(kP,kI,kD);
  bool IsValid = LinePoints[Increment][0] - position.x == 0 ? false : true;
  const float MAX_DIST = 40;
  if(angle == 420.69f){
    angle = !IsValid ? 0 : atan2(LinePoints[Increment][0]-position.x,LinePoints[Increment][1] - position.y);
  }else{
    angle = sensors.ToRadians(angle);
  }
  timer Count;
  Count.reset();
  while(Count.time(timeUnits::msec) < timeout){
    IsValid = LinePoints[Increment][0] - position.x == 0 ? false : true;
    float  TargetDirection = !IsValid ? 0 : atan2f(LinePoints[Increment][0]-position.x,LinePoints[Increment][1] - position.y);
    //a^2 + b^2 = c^2
    Config.Controller.Screen.setCursor(1, 1);
    Config.Controller.Screen.print("%f",TargetDirection);
    float  DistanceToTarget = !IsValid ? LinePoints[Increment][1] - position.y : sqrt((LinePoints[Increment][1] - position.y) * (LinePoints[Increment][1] - position.y) + (LinePoints[Increment][0]-position.x) * (LinePoints[Increment][0]-position.x));
    //err of angle and distance normalized to one
    float angleN = fabs(sensors.angleWrapRad(angle-position.angle))/PI,distanceN = fabs(DistanceToTarget/MAX_DIST);
    float angleSpeed;//variable to represent what % of motion should be angular
    angleSpeed = (angleN/(distanceN));
    if(angleSpeed > 1)//make sure no weird values get through
      angleSpeed = 1;
    if(angleSpeed < 0)
      angleSpeed = 0;
    if(sensors.angleWrapRad(angle-position.angle) < 0){//if the robot should turn left
      angleSpeed *= -1;
    }
    Move(sensors.ToDegrees(TargetDirection-position.angle),angleSpeed,SpeedControl.GetValue(angleN + distanceN));
    task::sleep(precision);
  }
  stop();
}
void Chassis::MoveTo(int speed, unsigned int timeout, unsigned int Increment, float angle){
  const int precision = 10;
  bool IsValid = LinePoints[Increment][0] - position.x == 0 ? false : true;
  const float MAX_DIST = 40;
  if(angle == 420.69f){
    angle = !IsValid ? 0 : atan2(LinePoints[Increment][0]-position.x,LinePoints[Increment][1] - position.y);
  }else{
    angle = sensors.ToRadians(angle);
  }
  float  DistanceToTarget;
  Config.Brain.Timer.reset();
  do{
    
    float  TargetDirection = !IsValid ? 0 : atan2(LinePoints[Increment][0]-position.x,LinePoints[Increment][1] - position.y);
    //a^2 + b^2 = c^2
    DistanceToTarget = !IsValid ? LinePoints[Increment][1] - position.y : sqrt((LinePoints[Increment][1] - position.y) * (LinePoints[Increment][1] - position.y) + (LinePoints[Increment][0]-position.x) * (LinePoints[Increment][0]-position.x));
    //err of angle and distance normalized to one
    float angleN = fabs(sensors.angleWrapRad(angle-position.angle))/PI,distanceN = fabs(DistanceToTarget/MAX_DIST);
    float angleSpeed;//variable to represent what % of motion should be angular
    angleSpeed = 1-(distanceN/(angleN+0.01));
    if(angleSpeed > 1)//make sure no weird values get through
      angleSpeed = 1;
    if(angleSpeed < 0)
      angleSpeed = 0;
    if(sensors.angleWrapRad(angle-position.angle) < 0){//if the robot should turn left
      angleSpeed *= -1;
    }
    Move(sensors.ToDegrees(TargetDirection-position.angle),angleSpeed,speed);
    task::sleep(precision);
  }while(Config.Brain.Timer.time(timeUnits::msec) < timeout && DistanceToTarget < 0.3f);
  stop();
}
void Chassis::TurnTo(float kP, float kI, float kD, unsigned int timeout, float angle){
  const int precision = 10;
  PID TurnControl(kP,kI,kD);
  float err = sensors.angleWrap(angle - sensors.ToDegrees(position.angle));
  Config.Brain.Timer.reset();
  while(Config.Brain.Timer.time(timeUnits::msec) < timeout){
    Move(0,err < 0 ? -1 : 1,fabs(TurnControl.GetValue(err)));
    err = sensors.angleWrap(angle - sensors.ToDegrees(position.angle));
    task::sleep(precision);
  }
  stop();
}
void Chassis::VisionAllign(bool isRed,unsigned int timeout){
  signature Ball = isRed ? Config.Vision1__RED_BALL : Config.Vision1__BLUE_BALL;
  Config.Vision1.takeSnapshot(Ball);
  if(Config.Vision1.objectCount == 0){
    return;
  }
  Config.Brain.Timer.reset();
  PID DriveControl(0.7,0,0);
  while((Config.Vision1.largestObject.centerX > 160 || Config.Vision1.largestObject.centerX < 156) && Config.Brain.timer(timeUnits::msec) < timeout){
    float err = Config.Vision1.largestObject.centerX - 158;//the x of the largest object - Vision sensors width/2
    Move(err < 0 ? -90 : 90, 0,DriveControl.GetValue(err));
    Config.Vision1.takeSnapshot(Ball);
    task::sleep(10);
  }
  intakes.spin(127);
  PID ForwardControl(1,0,0);
  while(Config.Vision1.largestObject.centerY <= 180  && Config.Brain.timer(timeUnits::msec) < timeout){
    float err = Config.Vision1.largestObject.centerY - 180;
    //the x of the largest object - Vision sensors width/2
    Move(err < 0 ? 0 : 180, 0,ForwardControl.GetValue(err));
    Config.Vision1.takeSnapshot(Ball);
    task::sleep(10);
    
  }
  intakes.stop();
  stop();
}
  /*USER FUNCTIONS*/

void Chassis::Arcade(float xAxis,float yAxis){
  int x = expoDrive(xAxis,1.7,1,2), y = expoDrive(yAxis,1.7,1,2);
  RFWheel.spin(directionType::fwd,y - x,velocityUnits::pct);
  RBWheel.spin(directionType::fwd,y - x,velocityUnits::pct);
  LFWheel.spin(directionType::fwd,y+x,velocityUnits::pct);
  LBWheel.spin(directionType::fwd,y+x,velocityUnits::pct);
}
void Chassis::Tank(float yAxisLeft, float yAxisRight){
  int left = expoDrive(yAxisLeft,1.7,1,2), right = expoDrive(yAxisRight,1.7,1,2);
  RFWheel.spin(directionType::fwd,right,velocityUnits::pct);
  RBWheel.spin(directionType::fwd,right,velocityUnits::pct);
  LFWheel.spin(directionType::fwd,left,velocityUnits::pct);
  LBWheel.spin(directionType::fwd,left,velocityUnits::pct);
}
void Chassis::Mechanum(float StrafeAxis,float TurnAxis,float yAxis)
{
  int Y = expoDrive(yAxis,1.7,1,2), Turn = expoDrive(TurnAxis,1.7,1,2),Strafe = -expoDrive(StrafeAxis,1.7,1,2);
  RFWheel.spin(directionType::fwd,Y-Turn-Strafe,velocityUnits::pct);
  RBWheel.spin(directionType::fwd,Y-Turn+Strafe,velocityUnits::pct);
  LFWheel.spin(directionType::fwd,Y+Turn+Strafe,velocityUnits::pct);
  LBWheel.spin(directionType::fwd,Y+Turn-Strafe,velocityUnits::pct);
}
void Chassis::stop(){
  RFWheel.stop(brakeType::brake);
  LFWheel.stop(brakeType::brake);
  RBWheel.stop(brakeType::brake);
  LBWheel.stop(brakeType::brake);
}