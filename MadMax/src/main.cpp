#include "vex.h"
#include "vex_global.h"
#include <chrono>
using namespace vex;

vex::controller controller1 = vex::controller();
vex::brain Brain;

vex::motor LeftFront = vex::motor(vex::PORT12);
vex::motor RightFront = vex::motor(vex::PORT19,true);
vex::motor LeftBack = vex::motor(vex::PORT13); 
vex::motor RightBack = vex::motor(vex::PORT20,true); 
vex::motor armLift = vex::motor(vex::PORT1);
vex::motor armLeft = vex::motor(vex::PORT4,true);//5
vex::motor armRight = vex::motor(vex::PORT6); //
vex::motor ramp = vex::motor(vex::PORT2);

vex::vision VisionSensor = vex::vision(vex::PORT1);

#include "vex.h"
#include "vex_units.h"
#include "VisionSensor.h"
#include <iostream>
#include <cmath>

using namespace vex;
using namespace std;
using namespace std::chrono;
competition Competition;

#pragma region globalVariables
//---------------Global Variables-----------------
double armAngle = 0; //deprecated, used in old versions
int intakeState = 0;
int manual = 0; //deprecated, used in old versions
int countr = 0;
int y = 0; //deprecatd, used in old versions
int rampState = 0;
//------------------------------------------------

//---------------Global Functions-----------------
void rampMode() {
  if(rampState == 0) {
    rampState = 1;
  } else if(rampState == 1) {
    rampState = 0;
  }
}
void intakeMode() {
  if(intakeState == 0) {
    intakeState = 1;
  } else if(intakeState == 1) {
    intakeState = 0;
  }
}
//------------------------------------------------
#pragma endregion

#pragma region HelperTools
//sets drivetrain velocity, default is 30%
void velocityset(double left = 30, double right = 30) { 
  LeftFront.setVelocity(left,vex::percentUnits::pct);
  LeftBack.setVelocity(left,vex::percentUnits::pct);
  RightFront.setVelocity(right,vex::percentUnits::pct);
  RightBack.setVelocity(right,vex::percentUnits::pct);
}
//converts a distance in centimeters to degrees of wheel rotations
double todeg(double cm) {
  double wheelDiameter = 10.16;
  double circumference = wheelDiameter * 3.141592;
  double degreesToRotate = 360 * cm/circumference;
  return degreesToRotate;
}
//prints the position of a specified motor
void printAngle(){
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(100,150,"Angle: %0.2lf",ramp.rotation(vex::rotationUnits::deg));
}
//as the name suggests; used in experimental features(acceleration)
double tocm(double deg) {
  double wheelDiameter = 10.16;
  double circumference = wheelDiameter * 3.141592;
  double cm = deg * circumference /360;
  return cm;
}
//makes the robot move forward/backward. Shorthand used in experimental branches
void move(vex::directionType type) {
  LeftFront.spin(type);
  RightFront.spin(type);
  LeftBack.spin(type);
  RightBack.spin(type);
}
//moves forward(or backward if cm<0) a certain distance
void forwardDistance(double cm, double vel) {
  velocityset(vel,vel);
  double deg = todeg(cm);
  LeftFront.rotateFor(deg, vex::rotationUnits::deg, false);
  LeftBack.rotateFor(deg, vex::rotationUnits::deg, false);
  RightFront.rotateFor(deg, vex::rotationUnits::deg, false);
  RightBack.rotateFor(deg, vex::rotationUnits::deg);
}
 //moves forward(or backward if cm<0) for a certain time
void forwardTime(double s, double vel) {
  velocityset(vel,vel);
  LeftFront.spin(vex::directionType::fwd);
  LeftBack.spin(vex::directionType::fwd);
  RightFront.spin(vex::directionType::fwd);
  RightBack.spin(vex::directionType::fwd);
  vex::task::sleep(s);
  LeftFront.stop();
  RightFront.stop();
  LeftBack.stop();
  RightBack.stop();
}
//makes the robot turn. "degrees" refers to wheel rotations, not the whole body
void turn(bool directn, double degrees, double vel) {
  velocityset(vel,vel);
  if(directn == 0) { // turns right
    LeftFront.rotateFor(degrees, vex::rotationUnits::deg, false);
    LeftBack.rotateFor(degrees, vex::rotationUnits::deg, false);
    RightFront.rotateFor(-degrees, vex::rotationUnits::deg, false);
    RightBack.rotateFor(-degrees, vex::rotationUnits::deg);
  } else {//turns left
    LeftFront.rotateFor(-degrees, vex::rotationUnits::deg, false);
    LeftBack.rotateFor(-degrees, vex::rotationUnits::deg, false);
    RightFront.rotateFor(degrees, vex::rotationUnits::deg, false);
    RightBack.rotateFor(degrees, vex::rotationUnits::deg);
  }
}
#pragma endregion

#pragma region actions
void tank() {
  LeftFront.spin(vex::directionType::fwd, controller1.Axis3.position()*0.75, vex::percentUnits::pct);
  LeftBack.spin(vex::directionType::fwd, controller1.Axis3.position()*0.75, vex::percentUnits::pct);
  RightFront.spin(vex::directionType::fwd, controller1.Axis2.position()*0.75, vex::percentUnits::pct);
  RightBack.spin(vex::directionType::fwd, controller1.Axis2.position()*0.75, vex::percentUnits::pct);
}
void arm() {
  /*
  These commands set up the initial values for the robot.
  The "setBrake(brakeType::hold)" commands for the ramp and armLift
  will make it so that executing a .stop() function will hold the motors in place(using power).
  The ramp and armLift are designated with 45% and 100% power respectively
  /**/  
  ramp.setBrake(brakeType::hold);
  armLift.setBrake(brakeType::hold);
  ramp.setVelocity(45,vex::percentUnits::pct);
  armLift.setVelocity(100,vex::percentUnits::pct);

  //------------manual mode---------------
  /*
  This is the manual mode. Modes are changed by the callback function rampMode.
   The robot initially starts in automatic mode but the button Y on the controller switches between the two modes.
  As the name suggests, in this mode all of the mechanisms are controlled manually.
  /**/
  if(rampState == 1) {
    controller1.Screen.clearScreen(); 
    controller1.Screen.print("Manual Mode");
    if(controller1.ButtonUp.pressing()) {
      ramp.spin(vex::directionType::fwd);
    } else if(controller1.ButtonDown.pressing()) {
      ramp.spin(vex::directionType::rev);
    } else {
      /*
      This bit of code is the only "automatic" part in manual mode. It is a remnant from the previous version
      of the code when the modes were combined(outlined previously). This is just here in case
      the driver forgets to return to automatic mode whenever he lifts the arms as the ramp is usually
      in the way. However, there is a flaw in the driver must wait about 1-2 seconds for the ramp to extend
      as this was the only option for the previous iteration's combined set of functions. The problem 
      disappears entirely in automatic mode.
      /**/
      if(armLift.rotation(vex::rotationUnits::deg) > 50) {
        countr = 1;
        /*
        "countr" sets a boundary at 50o of rotation. Because an infinite loop is being run, countr
         marks when the arm is raised and tells the ramp to incline a bit to move out of the way. 
         /**/
        ramp.rotateTo(350,vex::rotationUnits::deg,false);
      } else if(armLift.rotation(vex::rotationUnits::deg) < 50 && countr == 1) {
        /*
        Here the logic detects when the arm is lowered(<50o). If this is true, then the robot
        knows that the arm is lowered and that the ramp can be returned to its original position
        without getting in the way of the arm and vice versa. Countr is reset to 0.
        Countr redirects the ramp.stop() command, allowing ramp.spin() to allow manual control and
        preventing the code immediately following this comment from running when the arm isn't raised
        initially–thus lowering the ramp regardless of whether the user wants to or not. However,
        this is also the reason why the robot must wait for ramp.rotateTo(10,vex::rotationUnits::deg); 
        to finish, as if the logic simply went through, then countr would be 0 and also ramp.stop() would 
        prevent the command from running.
        /**/
        ramp.rotateTo(10,vex::rotationUnits::deg);
        countr = 0;
      } else {  
        ramp.stop();
      }
      ramp.stop(); //double check: may be unnecessary
    }
      if(controller1.ButtonR1.pressing()) {
      armLift.spin(vex::directionType::fwd);
    } else if(controller1.ButtonR2.pressing()) {
      armLift.spin(vex::directionType::rev);
    } else {
      armLift.stop();
    }
  //------------------Automatic Mode----------------
  /*
  This is the automatic mode. This is the default mode when the code is run. In this mode,
  the motors(except for the drivetrains) are set to rotate a specific amount, specifically:
  ramp ==> perpendicular or declined position
  armLift ==> rest, height of lowest tower, or height of middle tower
  /**/
  } else {
    //this section controls the movement of the ramp
    controller1.Screen.clearScreen();
    controller1.Screen.print("Automatic Mode");
    if(controller1.ButtonUp.pressing()) {
      ramp.rotateTo(1300,vex::rotationUnits::deg,false); //perpendicular position(stacking)
    } else if(controller1.ButtonDown.pressing()) {
      ramp.rotateTo(10,vex::rotationUnits::deg,false); //declined position(intake / rest)
    } else {
      //This controls the ramp when the arm is raised
      //Functions similarly to the one in manual control
      if(armLift.rotation(vex::rotationUnits::deg) > 30) {
        countr = 1;
        ramp.rotateTo(500,vex::rotationUnits::deg,false);
      } else if(armLift.rotation(vex::rotationUnits::deg) < 30 && countr == 1) {
        //"false" tag for this function now available, simplifying the issue a lot 
        ramp.rotateTo(10,vex::rotationUnits::deg,false); 
        countr = 0;
      }
      //notice the lack of .stop() methods
    }
    //Controls the raising of the arm
    if(controller1.ButtonX.pressing()) {
      armLift.rotateTo(1000,vex::rotationUnits::deg,false); //middle-height tower
    } else if(controller1.ButtonA.pressing()) {
      armLift.rotateTo(800,vex::rotationUnits::deg,false); //lowest tower
    } else if(controller1.ButtonB.pressing()) {
      armLift.rotateTo(10,vex::rotationUnits::deg,false); //rest
    }
  }
  printAngle();
  //Functions below run regardless of the mode
  if(controller1.ButtonLeft.pressing()) {
    intakeState = 0;
  } else if(controller1.ButtonRight.pressing()) {
    intakeState = 1;
  }

  if(controller1.ButtonL1.pressing()) { //makes the rollers intake blocks
    armLeft.setVelocity(100,vex::percentUnits::pct);
    armRight.setVelocity(100,vex::percentUnits::pct);
    armLeft.spin(vex::directionType::fwd);
    armRight.spin(vex::directionType::fwd);
  } else if(controller1.ButtonL2.pressing()) { //spins in reverse / outtake
    armLeft.setVelocity(100,vex::percentUnits::pct);
    armRight.setVelocity(100,vex::percentUnits::pct);
    armLeft.spin(vex::directionType::rev);
    armRight.spin(vex::directionType::rev);
  } else {
    if(intakeState == 0) { //intakeState changed by "intakeMode" seen at the bottom of the code
      armLeft.stop();
      armRight.stop();
    } else {
      armLeft.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
      armRight.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    }
  }
}
#pragma endregion

#pragma region auton
void skillsixteen() {
  //------------Expansion----------------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(500);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(300,-40);
  vex::task::sleep(150);
  //-------------Intake------------------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);
  forwardDistance(75,15);
  vex::task::sleep(500);
  forwardDistance(-10,20);
  vex::task::sleep(500);
  turn(0,147,20);
  vex::task::sleep(500);
  forwardDistance(-72,20);
  vex::task::sleep(500);
  turn(1,147,20);
  vex::task::sleep(500);


  forwardDistance(78,15);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  armLeft.stop();
  armRight.stop();
  vex::task::sleep(500);
  forwardDistance(-38,15);

  turn(1,380,20); //389
  vex::task::sleep(500);

  //--------------Stacking----------------
  forwardTime(2500,50);
  vex::task::sleep(500);

  forwardTime(500,-10);

  armLeft.setVelocity(25,vex::percentUnits::pct);
  armRight.setVelocity(25,vex::percentUnits::pct);
  armLeft.rotateFor(-75,vex::rotationUnits::deg,false);
  armRight.rotateFor(-75,vex::rotationUnits::deg);

  armLeft.stop();
  armRight.stop();

  vex::task::sleep(500);

  armLeft.setVelocity(20,vex::percentUnits::pct);
  armRight.setVelocity(20,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  ramp.setVelocity(30,vex::percentUnits::pct);
  ramp.rotateTo(1300,vex::rotationUnits::deg);
  vex::task::sleep(500);
  forwardTime(800,10);
  vex::task::sleep(500);
  armLeft.setVelocity(-15,vex::percentUnits::pct);
  armRight.setVelocity(-15,vex::percentUnits::pct);
  armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  armRight.rotateFor(-800,vex::rotationUnits::deg,false);
  forwardDistance(-30,30); // 25
  armLeft.stop();
  armRight.stop();

  //------------Scoring-------------
   vex::task::sleep(500);
  turn(1,430,15); //maybe 405
  LeftFront.setVelocity(30,vex::percentUnits::pct);
  LeftBack.setVelocity(30,vex::percentUnits::pct);
  RightFront.setVelocity(30,vex::percentUnits::pct);
  RightBack.setVelocity(30,vex::percentUnits::pct);
  vex::task::sleep(500);
  ramp.rotateTo(400,vex::rotationUnits::deg,false);
  forwardTime(2500,-50);
  vex::task::sleep(500);
  //----------------------------------------
  forwardDistance(100,25);
  vex::task::sleep(500);
  armLeft.setVelocity(40,vex::percentUnits::pct);
  armRight.setVelocity(40,vex::percentUnits::pct);
  armLeft.rotateFor(300,vex::rotationUnits::deg,false);
  armRight.rotateFor(300,vex::rotationUnits::deg);

  vex::task::sleep(500);
  forwardDistance(-10,20);
  vex::task::sleep(500);

  armLift.setVelocity(50,vex::percentUnits::pct);
  armLift.rotateTo(1100,vex::rotationUnits::deg);
  vex::task::sleep(200);
  forwardDistance(30,20);
  vex::task::sleep(500);
  armLeft.setVelocity(-15,vex::percentUnits::pct);
  armRight.setVelocity(-15,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  vex::task::sleep(2000);
  forwardDistance(-80,30);
  armLift.stop();
  armLeft.stop();
  armRight.stop();


}
void bluSafe() { //done
  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  //autonomous skills code. Code for the actual competition is yet to be
  //implemented as here the robot doesn’t start from a base.
  
  //-------------Expansion-------------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(300);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(300,-40);
  vex::task::sleep(150);
  //--------End of Expansion------------

  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);
  forwardDistance(110,30);
  vex::task::sleep(500);
  armLeft.rotateFor(-170,vex::rotationUnits::deg,false);
  armRight.rotateFor(-170,vex::rotationUnits::deg,false);

  forwardDistance(-35,30);
  vex::task::sleep(300);
  turn(1,400,20); //389
  vex::task::sleep(300);

  forwardTime(1000,60);
  vex::task::sleep(200);
  forwardTime(550,-10);

  ramp.setVelocity(45,vex::percentUnits::pct);
  ramp.rotateTo(1300,vex::rotationUnits::deg);
  vex::task::sleep(300);
  forwardTime(500,15);
  vex::task::sleep(500);
  armLeft.setVelocity(-20,vex::percentUnits::pct);
  armRight.setVelocity(-20,vex::percentUnits::pct);
  armLeft.rotateFor(800,vex::rotationUnits::deg,false);
  armRight.rotateFor(800,vex::rotationUnits::deg,false);
  ramp.setVelocity(100,vex::percentUnits::pct);
  ramp.rotateTo(400,vex::rotationUnits::deg,false);
  forwardDistance(-30,60); // 25
  armLeft.stop();
  armRight.stop();


}
void bluEight() {
  //---------------
  //expansion
  armLeft.setVelocity(100,vex::percentUnits::pct);
  ;armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(200);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(100,-40);
  // vex::task::sleep(100);

  //end of expansion
  //----------------intake---------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);

  forwardDistance(87,45);
  vex::task::sleep(200);
  turn(1,120,40);
  vex::task::sleep(200);
  forwardDistance(-90,50);
  vex::task::sleep(150);
  turn(0,143,40);
  vex::task::sleep(200);
  forwardDistance(85,30);
  vex::task::sleep(100);
  forwardDistance(-10,20);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  vex::task::sleep(100);
  turn(1,375,30);
  vex::task::sleep(150);


  armLeft.setVelocity(8,vex::percentUnits::pct);
  armRight.setVelocity(8,vex::percentUnits::pct);
  //---------------------------
  forwardTime(1100,95);
  vex::task::sleep(100);

  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  armLeft.setVelocity(-8,vex::percentUnits::pct);
  armRight.setVelocity(-8,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  ramp.setVelocity(50,vex::percentUnits::pct);

  ramp.rotateTo(800,vex::rotationUnits::deg);
  ramp.setVelocity(27,vex::percentUnits::pct);
  ramp.rotateTo(1200,vex::rotationUnits::deg);
  armLeft.stop();
  armRight.stop();
  vex::task::sleep(100);
  armLeft.setVelocity(-20,vex::percentUnits::pct);
  armLeft.rotateFor(800,vex::rotationUnits::deg,false);
  armRight.rotateFor(800,vex::rotationUnits::deg,false);
  ramp.setVelocity(100,vex::percentUnits::pct);
  ramp.rotateTo(400,vex::rotationUnits::deg,false);
  forwardDistance(-30,60); // 25
  armLeft.stop();
  armRight.stop();

}
//Red Side programs are usually mirrors. These are still being tested
void redSafe() {
  //autonomous skills code. Code for the actual competition is yet to be
  //implemented as here the robot doesn’t start from a base.
  
  //expansion
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(500);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(300,-40);
  vex::task::sleep(150);

  //end of expansion
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);
  forwardDistance(85,30);
  armLeft.stop();
  armRight.stop();
  vex::task::sleep(300);

  armLeft.setVelocity(15,vex::percentUnits::pct);
  armRight.setVelocity(15,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardDistance(-25,30);
  vex::task::sleep(500);
  turn(0,380,20); //389
  vex::task::sleep(300);
  // armLeft.setVelocity(-20,vex::percentUnits::pct);
  // armRight.setVelocity(-20,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // vex::task::sleep(1000);

  forwardTime(1200,70);
  vex::task::sleep(200);
  forwardTime(550,-10);

  armLeft.setVelocity(15,vex::percentUnits::pct);
  armRight.setVelocity(15,vex::percentUnits::pct);
  armLeft.rotateFor(-400,vex::rotationUnits::deg,false); // 130
  armRight.rotateFor(-400,vex::rotationUnits::deg,false); // 130

  ramp.setVelocity(40,vex::percentUnits::pct);
  ramp.rotateTo(1300,vex::rotationUnits::deg);
  vex::task::sleep(300);
  forwardTime(600,15);
  vex::task::sleep(300);
  armLeft.setVelocity(-15,vex::percentUnits::pct);
  armRight.setVelocity(-15,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  armRight.rotateFor(-800,vex::rotationUnits::deg,false);
  ramp.rotateTo(400,vex::rotationUnits::deg,false);
  forwardDistance(-45,40); // 25
  armLeft.stop();
  armRight.stop();
}
void redEight() {
  //---------------
  //expansion
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(200);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(100,-40);
  // vex::task::sleep(100);
  //end of expansion
  //----------------intake---------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);

  forwardDistance(87,45);
  vex::task::sleep(150);
  turn(0,120,40);
  vex::task::sleep(150);
  forwardDistance(-90,50);
  vex::task::sleep(150);
  turn(1,128,40);
  vex::task::sleep(150);
  forwardDistance(85,30);
  vex::task::sleep(100);
  forwardDistance(-10,25);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  vex::task::sleep(150);
  turn(0,348,30);
  vex::task::sleep(150);

  armLeft.setVelocity(8,vex::percentUnits::pct);
  armRight.setVelocity(8,vex::percentUnits::pct);
  //---------------------------
  forwardTime(1100,95);
  vex::task::sleep(100);


  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  armLeft.setVelocity(-18,vex::percentUnits::pct);
  armRight.setVelocity(-18,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  ramp.setVelocity(50,vex::percentUnits::pct);
  // ramp.rotateTo(700,vex::rotationUnits::deg);
  // armLeft.setVelocity(50,vex::percentUnits::pct);
  // armRight.setVelocity(50,vex::percentUnits::pct);
  // armLeft.rotateFor(500,vex::rotationUnits::deg,false);
  // armRight.rotateFor(500,vex::rotationUnits::deg,false);
  ramp.rotateTo(300,vex::rotationUnits::deg);
  armLeft.setVelocity(50,vex::percentUnits::pct);
  armRight.setVelocity(50,vex::percentUnits::pct);
  armLeft.rotateFor(400,vex::rotationUnits::deg,false);
  armRight.rotateFor(400,vex::rotationUnits::deg,false);
  ramp.rotateTo(700,vex::rotationUnits::deg);
  armLeft.stop(); armRight.stop();
  ramp.setVelocity(25,vex::percentUnits::pct);
  ramp.rotateTo(1200,vex::rotationUnits::deg);
  armLeft.stop();
  armRight.stop();
  // vex::task::sleep(100);
  // forwardTime(300,10);
  vex::task::sleep(200);
  armLeft.setVelocity(-20,vex::percentUnits::pct);
  armRight.setVelocity(-20,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  armLeft.rotateFor(800,vex::rotationUnits::deg,false);
  armRight.rotateFor(800,vex::rotationUnits::deg,false);
  ramp.setVelocity(100,vex::percentUnits::pct);
  ramp.rotateTo(400,vex::rotationUnits::deg,false);
  forwardDistance(-30,60); // 25
  armLeft.stop();
  armRight.stop();
}
void bluEightSlow() {

  //------------Expansion---------------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(500);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(300,-40);
  vex::task::sleep(150);

  //-------------Intake
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);
  forwardDistance(75,15);
  vex::task::sleep(500);
  forwardDistance(-10,20);
  vex::task::sleep(500);
  turn(0,135,20);
  vex::task::sleep(500);
  forwardDistance(-78,20);
  vex::task::sleep(500);
  turn(1,135,20);
  vex::task::sleep(500);
  //----------------
  forwardDistance(78,15);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  armLeft.stop();
  armRight.stop();
  vex::task::sleep(500);
  forwardDistance(-38,15);

  turn(1,380,20); //389
  vex::task::sleep(500);
  // armLeft.setVelocity(-20,vex::percentUnits::pct);
  // armRight.setVelocity(-20,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // vex::task::sleep(1000);

  forwardTime(2500,50);
  vex::task::sleep(500);

  forwardTime(500,-10);

  armLeft.setVelocity(25,vex::percentUnits::pct);
  armRight.setVelocity(25,vex::percentUnits::pct);
  armLeft.rotateFor(-50,vex::rotationUnits::deg,false);
  armRight.rotateFor(-50,vex::rotationUnits::deg);

  armLeft.stop();
  armRight.stop();

  vex::task::sleep(500);

  armLeft.setVelocity(20,vex::percentUnits::pct);
  armRight.setVelocity(20,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  ramp.setVelocity(30,vex::percentUnits::pct);
  ramp.rotateTo(1300,vex::rotationUnits::deg);
  vex::task::sleep(500);
  forwardTime(800,10);
  vex::task::sleep(500);
  armLeft.setVelocity(-15,vex::percentUnits::pct);
  armRight.setVelocity(-15,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  armRight.rotateFor(-800,vex::rotationUnits::deg,false);
  forwardDistance(-45,60); // 25
  armLeft.stop();
  armRight.stop();
}

void bluEightOld() {
   //---------------
  //expansion
  // armLeft.setVelocity(100,vex::percentUnits::pct);
  // armRight.setVelocity(100,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::rev);
  // armRight.spin(vex::directionType::rev);
  // vex::task::sleep(500);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // forwardTime(300,-40);
  // vex::task::sleep(150);

  //end of expansion
  //----------------
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);
  //intake
  forwardDistance(85,30);
  vex::task::sleep(100);
  forwardDistance(-30,40);
  vex::task::sleep(200);
  turn(0,180,20);
  vex::task::sleep(150);
  forwardDistance(-74,40);
  vex::task::sleep(250);
  turn(1,175,20);
  vex::task::sleep(150);
  //----------------
  //intake
  forwardDistance(90,30);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);

  armLeft.setVelocity(15,vex::percentUnits::pct);
  armRight.setVelocity(15,vex::percentUnits::pct);
  // vex::task::sleep(100);
  forwardDistance(-50,40);
  vex::task::sleep(400);
  turn(1,400,20); //389
  vex::task::sleep(150);
  // armLeft.setVelocity(-20,vex::percentUnits::pct);
  // armRight.setVelocity(-20,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // vex::task::sleep(1000);
  armLeft.stop();
  armRight.stop();
  // forwardTime(1500,65);
  // vex::task::sleep(200);

  // forwardTime(600,-10);

  // armLeft.setVelocity(20,vex::percentUnits::pct);
  // armRight.setVelocity(20,vex::percentUnits::pct);
  // armLeft.rotateFor(-100,vex::rotationUnits::deg,false);
  // armRight.rotateFor(-100,vex::rotationUnits::deg);

  // armLeft.setVelocity(20,vex::percentUnits::pct);
  // armRight.setVelocity(20,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // ramp.setVelocity(45,vex::percentUnits::pct);
  // ramp.rotateTo(1300,vex::rotationUnits::deg);
  // vex::task::sleep(100);
  // forwardTime(650,10);
  // vex::task::sleep(100);
  // armLeft.setVelocity(-15,vex::percentUnits::pct);
  // armRight.setVelocity(-15,vex::percentUnits::pct);
  // // armLeft.spin(vex::directionType::fwd);
  // // armRight.spin(vex::directionType::fwd);
  // armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  // armRight.rotateFor(-800,vex::rotationUnits::deg,false);
  // forwardDistance(-45,65); // 25
  // armLeft.stop();
  // armRight.stop();
  ramp.setVelocity(60,vex::percentUnits::pct);
  ramp.rotateTo(1300,vex::rotationUnits::deg,false);
  forwardTime(2500,60);
  vex::task::sleep(500);
  armLeft.setVelocity(-15,vex::percentUnits::pct);
  armRight.setVelocity(-15,vex::percentUnits::pct);
  armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  forwardDistance(-30,40); // 25
  armLeft.stop();
  armRight.stop();
}
void bluSafeSlow() {
    //autonomous skills code. Code for the actual competition is yet to be
  //implemented as here the robot doesn’t start from a base.
  
  //expansion
  // armLeft.setVelocity(100,vex::percentUnits::pct);
  // armRight.setVelocity(100,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::rev);
  // armRight.spin(vex::directionType::rev);
  // vex::task::sleep(500);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // forwardTime(300,-40);
  // vex::task::sleep(150);

  //end of expansion
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);//
  armRight.spin(vex::directionType::fwd);
  forwardDistance(83,20);
  armLeft.stop();
  armRight.stop();
  vex::task::sleep(500);

  armLeft.setVelocity(5,vex::percentUnits::pct);
  armRight.setVelocity(5,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardDistance(-23,60);
  vex::task::sleep(500);
  turn(1,380,20); //389
  vex::task::sleep(500);
  // armLeft.setVelocity(-20,vex::percentUnits::pct);
  // armRight.setVelocity(-20,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // vex::task::sleep(1000);

  forwardTime(1500,70);
  vex::task::sleep(200);
  forwardTime(550,-10);

  armLeft.setVelocity(25,vex::percentUnits::pct);
  armRight.setVelocity(25,vex::percentUnits::pct);
  armLeft.rotateFor(-450,vex::rotationUnits::deg,false); // 130
  armRight.rotateFor(-450,vex::rotationUnits::deg); // 130

  vex::task::sleep(500);

  armLeft.setVelocity(15,vex::percentUnits::pct);
  armRight.setVelocity(15,vex::percentUnits::pct);
  ramp.setVelocity(60,vex::percentUnits::pct);
  ramp.rotateTo(750,vex::rotationUnits::deg);
  vex::task::sleep(500);
  armLeft.rotateFor(300,vex::rotationUnits::deg,false); // 130
  armRight.rotateFor(300,vex::rotationUnits::deg);
  vex::task::sleep(500);
  ramp.rotateTo(1300,vex::rotationUnits::deg);
  vex::task::sleep(500);
  forwardTime(800,15);
  vex::task::sleep(800);
  // armLeft.setVelocity(-15,vex::percentUnits::pct);
  // armRight.setVelocity(-15,vex::percentUnits::pct);
  // armLeft.spin(vex::directionType::fwd);
  // armRight.spin(vex::directionType::fwd);
  // armLeft.rotateFor(-800,vex::rotationUnits::deg,false);
  // armRight.rotateFor(-800,vex::rotationUnits::deg,false);
  forwardDistance(-45,30); // 25
  armLeft.stop();
  armRight.stop();
  vex::task::sleep(1000);
}
//One point autonomous code if we ever need it
void one() {
  armLeft.setVelocity(100,vex::percentUnits::pct);
  armRight.setVelocity(100,vex::percentUnits::pct);
  armLeft.spin(vex::directionType::rev);
  armRight.spin(vex::directionType::rev);
  vex::task::sleep(500);
  armLeft.spin(vex::directionType::fwd);
  armRight.spin(vex::directionType::fwd);
  forwardTime(300,-40);
  vex::task::sleep(150);
  armLeft.stop();
  armRight.stop();
  forwardTime(5000,30);
  forwardTime(2000,-30);
}
#pragma endregion

#pragma region compTemplate
void pre_auton(void) {
  armLift.resetRotation();
  ramp.resetRotation();
}

void usercontrol() {
  controller1.ButtonY.pressed(rampMode);
  controller1.ButtonRight.pressed(intakeMode);
  while(true) {
    arm();
    tank();
    vex::task::sleep(10);
  }
  /* Used to output the time taken for the code to finish. It's in usercontrol
    because sometimes it's too hassle to grab a physical switch and usercontrol
    always runs by default on the remote.
  auto start = high_resolution_clock::now(); 
  bluSafe(); //put auton code here
  auto stopp = high_resolution_clock::now(); 
  auto duration = duration_cast<milliseconds>(stopp - start);
  cout << "Time taken by function: "
         << duration.count()<< " milliseconds" << endl; 
  */
}
void autonomous(void) {
  bluSafe();
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while(1) {
    vex::task::sleep(100);
  }

}
#pragma endregion