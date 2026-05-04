/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\stuar                                            */
/*    Created:      Fri Apr 24 2026                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    15, 14, 13, 12  
// Controller1          controller                    
// Arm                  motor         11              
// Claw                 motor         1               
// LimitSwitchArm       limit         A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

extern bool RemoteControlCodeEnabled;

int main() {
  vexcodeInit();


  Arm.setBrake(hold);
  Claw.setBrake(brake);
  Claw.setVelocity(100, percent);

  // disable everything during homing
  RemoteControlCodeEnabled = false;
  Brain.Screen.clearScreen();
  Brain.Screen.print("Calibrating...");

  Arm.setVelocity(80, percent);
  Arm.spin(forward);
  wait (100, msec);
  Arm.stop();

  Arm.setVelocity(3.5, percent);
  Arm.spin(reverse);
  waitUntil(LimitSwitchArm.pressing());
  Arm.stop();
  Arm.resetPosition();

  Brain.Screen.clearScreen();
  Brain.Screen.print("Ready!");
  Claw.setVelocity(50, percent);

  // re-enable driver control
  RemoteControlCodeEnabled = true;

  
}
