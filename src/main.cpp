#pragma once
#include <robot-config.h>
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

using namespace vex;

brain Brain;
competition Competition;
ai::jetson jetson_comms;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
// #define MANAGER_ROBOT 1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link link(PORT9, "robot_32456_1", linkType::manager);
motor left_side = motor(1);
motor right_side = motor(4);
VB_Drive drive(left_side, right_side, 7);
Arm arm(8, 9);
Clamp clamp(Brain.ThreeWirePort.A);
Intake intake(10, 11);
void periodic()
{
  while (1)
  {
    arm.periodic();
    clamp.periodic();
    intake.periodic();
  }
}
thread periodicThread = thread(periodic);

#else
#pragma message("building for the worker")
ai::robot_link link(PORT2, "robot_32456_1", linkType::worker);

#endif

motor fl(2, false);
motor fr(10);
motor bl(11, false);
motor br(16);
motor_group left_side(fl, bl);
motor_group right_side(fr, br);
// VB_Drive drive(left_side, right_side, 20);
gps GPS(PORT19);
smartdrive drive(left_side, right_side, GPS);
int marker = 0;
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
  GPS.calibrate();
  while (GPS.isCalibrating())
  {
    wait(5, msec);
    marker++;
  }

  // drive.drive.turn(turnType::left);

  repeat(50)
  {
    marker++;
  }
  drive.turnToHeading(180, degrees);
  drive.stop();
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
  // Add functions for interaction phase
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

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