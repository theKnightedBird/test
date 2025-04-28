/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
// Robot configuration code.

// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson jetson_comms;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define MANAGER_ROBOT 1
// Change to true
bool isRed = false;

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link link(PORT10, "robot_32456_1", linkType::manager);
// things for 24-inch
motor left1 = motor(14, ratio6_1, false);
motor left2 = motor(19, ratio6_1, true);
motor left3 = motor(20, ratio6_1, true);
motor_group leftDrive = motor_group(left1, left2);
motor right1 = motor(15, ratio6_1, false);
motor right2 = motor(16, ratio6_1, true);
motor right3 = motor(17, ratio6_1, false);
motor_group rightDrive = motor_group(right2);
gps GPS = gps(PORT7, -127, -165, distanceUnits::mm, 180);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 1);
digital_out clamp = digital_out(Brain.ThreeWirePort.G);
digital_out doinker = digital_out(Brain.ThreeWirePort.H);
motor intake = motor(PORT8, ratio18_1, false);
#else
#pragma message("building for the worker")
// things for 15-inch
motor left1 = motor(18, ratio18_1, false);
motor left2 = motor(19, ratio18_1, true);
motor left3 = motor(20, ratio18_1, true);
motor_group leftDrive = motor_group(left1, left2, left3);
motor right1 = motor(15, ratio18_1, false);
motor right2 = motor(16, ratio18_1, true);
motor right3 = motor(17, ratio18_1, false);
motor_group rightDrive = motor_group(left1, left2, left3);
gps GPS = gps(PORT7, -127, -165, distanceUnits::mm, 180);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 1);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
digital_out doinker = digital_out(Brain.ThreeWirePort.B);
motor intake = motor(PORT8, ratio18_1, false);
ai::robot_link link(PORT10, "robot_32456_1", linkType::worker);
#endif

// functions

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void)
{
  // Calibrate GPS Sensor
  GPS.calibrate();
  // Optional wait to allow for calibration
  waitUntil(!(GPS.isCalibrating()));

  // // Finds and moves robot to over the closest blue ring
  // goToObject(OBJECT::BlueRing);
  // grabRing();
  // // Find and moves robot to the closest mobile drop
  // // then drops the ring on the goal
  // goToObject(OBJECT::MobileGoal);
  // dropRing();
  // // Back off from the goal
  // Drivetrain.driveFor(-30, distanceUnits::cm);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Interaction(void)
{
  Drivetrain.setDriveVelocity(90, percentUnits::pct);
  Drivetrain.drive(forward);
  // while (true)
  // {
  //   grab_goal();
  //   repeat(6)
  //   {
  //     score_ring();
  //   }

  //   drop_in_corner();
  // }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = false;

void autonomousMain(void)
{
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period.
  // ..........................................................................

  if (firstAutoFlag)
    auto_Isolation();
  else
    auto_Interaction();

  firstAutoFlag = false;
}

int main()
{

  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  // FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  while (1)
  {
    // get last map data
    jetson_comms.get_data(&local_map);

    // set our location to be sent to partner robot
    link.set_remote_location(local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status);

    // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

    // request new data
    // NOTE: This request should only happen in a single task.
    jetson_comms.request_map();

    // Allow other tasks to run
    this_thread::sleep_for(loop_time);
  }
}