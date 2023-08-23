/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision14             vision        14              
// Inertial             inertial      12              
// EncoderA             encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----
/* CONTROLS */
//INTAKE L1 Intake-front L2 Outake-front R1 Intake-tower R2 Outake-tower
//toggle X
//drive: Mechanum
#include "vex.h"
#include "Chassis.h"
using namespace vex;

//field constants
const float FIELD_WIDTH = 140.5;
const float TILE_WIDTH = FIELD_WIDTH/6.0;
//Robot Constant
//defining an instance of each class
Chassis Drive;
//Manual/Preset Control
bool isManual = true;
//what colour the brain is
color clearColor = color::blue;
//amount of autos
const unsigned int autoSize = 5;
//which auto
int autoSelector = 1;
// A global instance of competition
competition Competition;

//clears all values on the brain
void clearAll();

//prints all values to the brain
void printAll();

//debug task
int debug();

//reformatted functions for button.pressed


//initialize and configurate objects and values
void init();

//chooses auto
int selectAuto();
bool IsRed();
//Vex code doesnt like member functions
int a(){
  while(1){
    Drive.trackPosition();
  }
}

bool runB = false;
int b(){
  PID a(4.1,0.01,6,127);
  tower.preset_selector = 0;
  while(true){
    if(runB){
      tower.spin(a.GetValue(tower.Preset[tower.preset_selector]-tower.GetRotation()));
    }
    task::sleep(20);
  }
  return 1;
}
//declare tasks
task DebugTask(debug,3);
task AutoSelectTask(selectAuto,4);
task trackingTask(a,1);
task ArmTask(b,2);
void switchManual(){
  isManual = !isManual;
}
void pickUpRing(){
  if( tower.GetRotation() > tower.Preset[1]){return;}

  bool temp = runB;
  runB = false;
  tower.spin(-127);
  intakes.spin(127);
  unsigned long time = Config.Brain.timer(timeUnits::msec);
  while(Config.Brain.timer(timeUnits::msec) - time < 500){
    if(tower.GetRotation() <= tower.Preset[0]-3){
      break;
    }
    task::sleep(10);
  }
  tower.spin(127);
  while(Config.Brain.timer(timeUnits::msec) - time < 1000){
    if(tower.GetRotation() > tower.Preset[0]){
      break;
    }
    task::sleep(10);
  }
  intakes.stop();
  tower.stop();
  runB = temp;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/ 

void autonomous(void) {//29 50 60 71 101
  AutoSelectTask.stop();
  if(autoSelector == 1){//blue Right
    vector<array<float,3>> MainPos =
    {
      {4.5f*TILE_WIDTH,9,180},//start 
      {TILE_WIDTH*4.5f,TILE_WIDTH*1.5f,180},//forward a bit to match diagonal path of middle mogo
      {TILE_WIDTH*3.2f,TILE_WIDTH*2.8f,135},//position of middle mogo
      {TILE_WIDTH*4.5f,TILE_WIDTH*1.5f,135},//position for spinning
      {TILE_WIDTH*5.5f,TILE_WIDTH*0.5f,315},//position of corner
      //unlock and drop mogo
      {TILE_WIDTH*4.5f,TILE_WIDTH*1.5f,315},// : 5
      {TILE_WIDTH*5.5f,TILE_WIDTH*2.5f,180},//grab alliance mogo
      {TILE_WIDTH*5.5f,TILE_WIDTH*1.2f,180},
    };
    Drive.setX(MainPos[0][0]);
    Drive.setY(MainPos[0][1]);
    Drive.setAngle(MainPos[0][2]);
    Drive.setPoints(MainPos);
    runB = true;
    tower.preset_selector = 0;
    tower.Free();
    //4.5s
    Drive.PIDMoveTo(220,0,0, 1200, 1, MainPos[1][2]);
    Drive.PIDMoveTo(200,0,0, 800, 1, MainPos[2][2]);
    Drive.PIDMoveTo(220,0,0, 2500, 2, MainPos[2][2]);
    tower.Lock();
    //4.3s
    Drive.PIDMoveTo(220,0,0, 2500, 3, MainPos[3][2]);
    Drive.PIDMoveTo(220,0,0, 800, 3, MainPos[4][2]);
    Drive.PIDMoveTo(220,0,0, 2000, 4, MainPos[4][2]);
    tower.Free();
    //3.5s
    Drive.PIDMoveTo(220,0,0, 1000, 5, MainPos[5][2]);
    Drive.PIDMoveTo(220,0,0, 2500, 6, MainPos[6][2]);
    runB=true;
    tower.preset_selector = 1;
    //1.5s
    Drive.PIDMoveTo(220,0,0, 1000, 7, MainPos[7][2]);
    intakes.rotateFor(800, -127);
  }
  else if(autoSelector == 2){//blue Left
    vector<array<float,3>> MainPos =
    {
      {0,0,0},//start
    };
    Drive.setX(MainPos[0][0]);
    Drive.setY(MainPos[0][1]);
    Drive.setAngle(MainPos[0][2]);
    Drive.setPoints(MainPos);
    Drive.PIDMoveTo(220,0,0, 100000, 0, 0);
  }
  else if(autoSelector == 3){//Red Right
     vector<array<float,3>> MainPos =
    {
      {4.5f*TILE_WIDTH,9,180},//start 
      {TILE_WIDTH*4.5f,TILE_WIDTH*1.5f,180},//forward a bit to match diagonal path of middle mogo
      {TILE_WIDTH*3.2f,TILE_WIDTH*2.8f,135},//position of middle mogo
      {TILE_WIDTH*4.5f,TILE_WIDTH*1.5f,135},//position for spinning
      {TILE_WIDTH*5.5f,TILE_WIDTH*0.5f,315},//position of corner
      //unlock and drop mogo
      {TILE_WIDTH*4.5f,TILE_WIDTH*1.5f,315},// : 5
      {TILE_WIDTH*5.5f,TILE_WIDTH*2.5f,180},//grab alliance mogo
      {TILE_WIDTH*5.5f,TILE_WIDTH*1.2f,180},
    };
    Drive.setX(MainPos[0][0]);
    Drive.setY(MainPos[0][1]);
    Drive.setAngle(MainPos[0][2]);
    Drive.setPoints(MainPos);
    runB = true;
    tower.preset_selector = 0;
    tower.Free();
    task::sleep(800);
    //4.5s
    Drive.PIDMoveTo(220,0,0, 1200, 1, MainPos[1][2]);
    Drive.PIDMoveTo(200,0,0, 800, 1, MainPos[2][2]);
    Drive.PIDMoveTo(220,0,0, 2500, 2, MainPos[2][2]);
    tower.Lock();
    //4.3s
    Drive.PIDMoveTo(220,0,0, 2500, 3, MainPos[3][2]);
    Drive.PIDMoveTo(220,0,0, 800, 3, MainPos[4][2]);
    Drive.PIDMoveTo(220,0,0, 2000, 4, MainPos[4][2]);
    tower.Free();
    //3.5s
    Drive.PIDMoveTo(220,0,0, 1000, 5, MainPos[5][2]);
    Drive.PIDMoveTo(220,0,0, 2500, 6, MainPos[6][2]);
    runB=true;
    tower.preset_selector = 1;
    //1.5s
    Drive.PIDMoveTo(220,0,0, 1000, 7, MainPos[7][2]);
    intakes.rotateFor(800, -127);
  }
  else if(autoSelector == 5){//skills
  vector<array<float,3>> MainPos = // TODO :: CHECK ARM ANGLES FOR MOGOS
    {
      {9,TILE_WIDTH*0.5f,270},//start : 0
      {TILE_WIDTH*1.5f,TILE_WIDTH*0.5f,270},//grab the blue MOGO and locc : 1
      {TILE_WIDTH,TILE_WIDTH*0.5f,270},// : 2
      {TILE_WIDTH*1.5f,TILE_WIDTH*2,0},// prep angle : 3
      {TILE_WIDTH*1.5f,TILE_WIDTH*4.8f,0},//PUUUUUUUUUSH : 4

      {TILE_WIDTH*1.5f,TILE_WIDTH*4,180},//stacko #1 : PATH 5-6 : 5
      {TILE_WIDTH*3,TILE_WIDTH*5-10,180},// : 6 slap that boi down on the platform

      {TILE_WIDTH*1.5f,TILE_WIDTH*4,180},//Drive back to line up for mogo 2 :7
      {TILE_WIDTH*1.5f,TILE_WIDTH*5,180},//Drive and grab the mogo LOCC : 8

      {TILE_WIDTH*1.5f,TILE_WIDTH*4,180},//stacko #2 : PATH 10-11 : 9
      {TILE_WIDTH*2.6f,TILE_WIDTH*5-10,180},// : slap that boi down on the platform : 10
    };
    Drive.setX(MainPos[0][0]);
    Drive.setY(MainPos[0][1]);
    Drive.setAngle(MainPos[0][2]);
    Drive.setPoints(MainPos);
    runB = true;
    tower.preset_selector = 0;
    tower.Free();
    task::sleep(800);
    Drive.PIDMoveTo(220,0,0, 2000, 1, MainPos[1][2]);//AND DRIVE AND TAKE THE FIRST MOGO
    tower.Lock();//YOINK --- MINE NOW

    Drive.PIDMoveTo(220,0,0, 1000, 2, MainPos[2][2]);//go back a bit
    Drive.PIDMoveTo(220,0,0, 3000, 3, MainPos[3][2]);
    Drive.PIDMoveTo(220,0,0, 4000, 4, MainPos[4][2]);//CHARGE

    /*stacking the first two mogos*/
    tower.preset_selector = 3;
    Drive.PIDMoveTo(220,0,0, 4000, 5, MainPos[5][2]);
    task::sleep(1000);
    Drive.PIDMoveTo(220,0,0, 4000, 6, MainPos[6][2]);
    tower.preset_selector = 1;
    tower.Free();
    task::sleep(1000);
    Drive.PIDMoveTo(220,0,0, 4000, 7, MainPos[7][2]);
    tower.preset_selector = 0;
    task::sleep(2000);
    Drive.PIDMoveTo(220,0,0, 4000, 8, MainPos[8][2]);
    tower.Lock();
    tower.preset_selector = 3;
    Drive.PIDMoveTo(220,0,0, 4000, 9, MainPos[9][2]);
    Drive.PIDMoveTo(220,0,0, 4000, 10, MainPos[10][2]);
    //MAXIMUM HEIGHT FOR STACKO
    tower.Free();//LET GO
    runB = true;
    tower.preset_selector = 1;//STACKO 🚓
    
    //Drive.MoveTo(127, unsigned int timeout, unsigned int Increment, float angle)
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {

  bool 👀 = true;
  // User control code here, inside the loop
  while (1) {
    //call the arcade drive code
    
    if(tower.GetRotation() > tower.Preset[2]){
      Drive.Mechanum(Config.Controller.Axis4.value()/2, Config.Controller.Axis1.value()/2, Config.Controller.Axis3.value()/2);
    }else{
      Drive.Mechanum(Config.Controller.Axis4.value(), Config.Controller.Axis1.value(), Config.Controller.Axis3.value());
    }
    //intake control
    if(Config.Controller.ButtonR1.pressing()){
      if(tower.GetRotation() < tower.Preset[1]){
        intakes.spin(127);
      }else{
        intakes.spin(-127);
      }
    }
    else
    {
      intakes.stop();
    }
    if(Config.Controller.ButtonR2.pressing()){
      pickUpRing();
    }
    //intake control
    if(isManual){
      if(Config.Controller.ButtonL2.pressing()){
        runB = false;
        tower.spin(-127);
        tower.Free();
      }
      else if(Config.Controller.ButtonL1.pressing()){
        runB = false;
        tower.spin(127);
        tower.Lock();
      }
      else if(tower.GetRotation() > tower.Preset[0] && tower.GetRotation() < tower.Preset[0] + 5){
        runB = true; tower.preset_selector = 0;
      }else
      {
        runB = false;
        tower.stop();
      }
    }else{
      runB = true;
      if(Config.Controller.ButtonL1.pressing() && tower.preset_selector < 3 && 👀){
        tower.Free();
        ++tower.preset_selector;
        👀 = false;
      }
      else if(Config.Controller.ButtonL2.pressing() && tower.preset_selector > 0 && 👀){
        tower.Lock();
        --tower.preset_selector;
        👀 = false;
      }
      else if (!Config.Controller.ButtonL1.pressing() && !Config.Controller.ButtonL2.pressing()) {
        👀 = true;
      }
    }
    task::sleep(2);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  init();

  //auto selection

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    task::sleep(100);
  }
}
void clearAll(){
  Config.Brain.Screen.clearScreen(clearColor);
}
void printAll(){
  Config.Brain.Screen.setCursor(1,1);
  Config.Brain.Screen.print("angle: %f",Drive.sensors.ToDegrees(Drive.getAngle()));
  Config.Brain.Screen.setCursor(2,1);
  Config.Brain.Screen.print("x coordinate: %f",Drive.getX());
  Config.Brain.Screen.setCursor(3,1);
  Config.Brain.Screen.print("y coordinate: %f",Drive.getY());
  Config.Brain.Screen.setCursor(4,1);
  Config.Brain.Screen.print("Left Encoder: %f",Config.LeftEncoder.position(rotationUnits::deg));
  Config.Brain.Screen.setCursor(5,1);
  Config.Brain.Screen.print("Right Encoder: %f",Config.RightEncoder.position(rotationUnits::deg));
  Config.Brain.Screen.setCursor(6,1);
  Config.Brain.Screen.print("Back Encoder: %f",Config.BackEncoder.position(rotationUnits::deg));
  Config.Brain.Screen.setCursor(7,1);
  Config.Brain.Screen.print("tower preset: %d",tower.preset_selector);
  Config.Brain.Screen.setCursor(8,1);
  Config.Brain.Screen.print("Current of spinny boy: %f",intakes.NeedleIntake.voltage());
  Config.Brain.Screen.setCursor(9,1);
  Config.Brain.Screen.print("Velocity of spinny boy: %f",intakes.NeedleIntake.velocity(percentUnits::pct));
  Config.Brain.Screen.setCursor(10,1);
  Config.Brain.Screen.print("arm rotation: %d",tower.GetRotation());
  Config.Brain.Screen.render();
}
int debug(){
  while(1){
    printAll();
    task::sleep(200);
    clearAll();
  }
}
void init(){
  Config.Brain.Screen.setCursor(6,1);
  Config.Brain.Screen.print("Test 1");
  Config.Vision1.setBrightness(150);
  Config.Optical.setLight(ledState::on);
  tower.SUPPOSED_TO_BE_A_PISTON.setBrake(brakeType::hold);
  DebugTask.resume();
  AutoSelectTask.resume();
  trackingTask.resume();
  ArmTask.resume();
  Config.Controller.ButtonX.pressed(switchManual);


  Drive.sensors.calibrateGyro();
  Config.LeftEncoder.setPosition(0, degrees);
  Config.RightEncoder.setPosition(0, degrees);
  Config.BackEncoder.setPosition(0, degrees);

}

int selectAuto(){
  Config.Controller.Screen.setCursor(2,1);
  switch(autoSelector){
      case 1: Config.Controller.Screen.print("BLUE R");break;
      case 2: Config.Controller.Screen.print("BLUE L");break;
      case 3: Config.Controller.Screen.print("RED R");break;
      case 4: Config.Controller.Screen.print("RED L");break;
      case 5: Config.Controller.Screen.print("Skills");break;
      default: Config.Controller.Screen.print("ERR: Invalid case argument");break;
  }
  while(1){
    //wait until left or right button is pressing
    while(!(Config.Controller.ButtonLeft.pressing() || Config.Controller.ButtonRight.pressing())){
      task::sleep(100);
    }
    //prep screen
    Config.Controller.Screen.setCursor(2,1);

    //if button left is pressing, decrease autoselector by one
    if(Config.Controller.ButtonLeft.pressing()){
      autoSelector = autoSelector <= 1 ? autoSize : autoSelector - 1;
      Config.Controller.Screen.clearLine(2);
    }

    //if button right is pressing, increase autoselector by one
    if(Config.Controller.ButtonRight.pressing()){
      autoSelector = autoSelector >= autoSize ? 1 : autoSelector + 1;
      Config.Controller.Screen.clearLine(2);
    }

    //print message depending on autoselector
    switch(autoSelector){
      case 1: Config.Controller.Screen.print("BLUE R");break;
      case 2: Config.Controller.Screen.print("BLUE L");break;
      case 3: Config.Controller.Screen.print("RED R");break;
      case 4: Config.Controller.Screen.print("RED L");break;
      case 5: Config.Controller.Screen.print("Skills");break;
      default: Config.Controller.Screen.print("ERR: Invalid case argument");break;
    }
    while(Config.Controller.ButtonLeft.pressing() || Config.Controller.ButtonRight.pressing()){
      task::sleep(100);
    }
  }
}