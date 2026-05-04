#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT15, ratio6_1, true);
motor leftMotorB = motor(PORT14, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT13, ratio6_1, false);
motor rightMotorB = motor(PORT12, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 219.44, 387.5, 311.12, mm, 0.6);
controller Controller1 = controller(primary);
motor Arm = motor(PORT11, ratio18_1, true);
motor Claw = motor(PORT1, ratio18_1, false);
limit LimitSwitchArm = limit(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis4
      // right = Axis3 - Axis4
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis4.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis4.position();
      int armSpeed = Controller1.Axis2.position();
      double armMin = 12;    // degrees - at limit switch
      double armMax = 740;  // degrees - tune this to your robot
      
      // check if the values are inside of the deadband range
      if (abs(drivetrainLeftSideSpeed) < 5 && abs(drivetrainRightSideSpeed) < 5) {
        // check if the motors have already been stopped
        if (DrivetrainNeedsToBeStopped_Controller1) {
          // stop the drive motors
          LeftDriveSmart.stop();
          RightDriveSmart.stop();
          // tell the code that the motors have been stopped
          DrivetrainNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the motors next time the input is in the deadband range
        DrivetrainNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }



      double armPos = Arm.position(degrees);
      double slowZone = 300; // degrees before limit to start slowing
      int effectiveSpeed = armSpeed;

      if (armPos > armMax - slowZone) {
        int calculated = (int)((armMax - armPos) / slowZone * 100);
        effectiveSpeed = armSpeed < calculated ? armSpeed : calculated;
      }
      if (armPos < armMin + slowZone) {
        int calculated = (int)(-(armPos - armMin) / slowZone * 100);
        effectiveSpeed = armSpeed > calculated ? armSpeed : calculated;
      }

      if (Controller1.ButtonR1.pressing() && armPos < armMax) {
        int btnSpeed = (armPos > armMax - slowZone) ? (int)((armMax - armPos) / slowZone * 100) : 100;
        Arm.setVelocity(btnSpeed, percent);
        Arm.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing() && armPos > armMin) {
        int btnSpeed = (armPos < armMin + slowZone) ? (int)((armPos - armMin) / slowZone * 100) : 100;
        Arm.setVelocity(btnSpeed, percent);
        Arm.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (abs(effectiveSpeed) > 5 && armPos < armMax && armSpeed > 0) {
        Arm.setVelocity(effectiveSpeed, percent);
        Arm.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (abs(effectiveSpeed) > 5 && armPos > armMin && armSpeed < 0) {
        Arm.setVelocity(-effectiveSpeed, percent);
        Arm.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Arm.stop();
        Controller1RightShoulderControlMotorsStopped = true;
      }

      Claw.setVelocity(100, percent);

      if (Controller1.ButtonL1.pressing()) {
        Claw.spin(forward);
      }

      if (!Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing()) {
        Claw.stop();
      }


      if (Controller1.ButtonL2.pressing()) {
        Claw.spin(reverse);
      }

      if (Controller1.ButtonUp.pressing() && Controller1.ButtonDown.pressing() && Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
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



      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Arm: %.0f", Arm.position(degrees));
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Claw: %.0f", Claw.position(degrees));

    }
    // wait before repeating the process
    wait(10, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}