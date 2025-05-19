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
#include "robot-config.h"

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
bool isRed = true;

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link link(PORT15, "robot_32456_1", linkType::manager);
// things for 24-inch
motor leftMotorA = motor(PORT4, ratio6_1, true);
motor leftMotorB = motor(PORT3, ratio6_1, true);
motor leftMotorC = motor(PORT1, ratio6_1, true);
motor rightMotorA = motor(PORT14, ratio6_1, false);
motor rightMotorB = motor(PORT12, ratio6_1, false);
motor rightMotorC = motor(PORT11, ratio6_1, false);
motor_group leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor_group rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC);
gps GPS = gps(PORT17, 100, 0, distanceUnits::mm, 90);
vantadrive Drivetrain = vantadrive(leftDrive, rightDrive, GPS);
digital_out clamp = digital_out(Brain.ThreeWirePort.B);
digital_out doinker = digital_out(Brain.ThreeWirePort.C);
motor intake = motor(PORT19, ratio18_1, false);
motor arm = motor(PORT7, ratio18_1, true);

#else
#pragma message("building for the worker")
// things for 15-inch
motor leftMotorA = motor(PORT7, ratio6_1, true);
motor leftMotorB = motor(PORT8, ratio6_1, true);
motor leftMotorC = motor(PORT9, ratio6_1, true);
motor rightMotorA = motor(PORT4, ratio6_1, false);
motor rightMotorB = motor(PORT5, ratio6_1, false);
motor rightMotorC = motor(PORT6, ratio6_1, false);
motor_group leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor_group rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial imu = inertial(PORT3);
gps GPS = gps(PORT18, -55, 50, distanceUnits::mm, 0.0);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, imu, 219.43, 254, 254, mm, 1);

digital_out clamp = digital_out(Brain.ThreeWirePort.A);
digital_out doinker = digital_out(Brain.ThreeWirePort.B);

motor intakeMotorA = motor(PORT2, ratio18_1, false);
motor intakeMotorB = motor(PORT10, ratio18_1, true);
motor_group intake = motor_group(intakeMotorA, intakeMotorB);

ai::robot_link link(PORT20, "robot_32456_1", linkType::worker);
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
  // Drivetrain.driveFor(100, mm);
  //  intake.spin(forward);
  // Drivetrain.setSpeeds(200.0, 200.0);
  // Drivetrain.turn(0.0);
  // Drivetrain.pointTowards(0.0, 0.0);
  // Drivetrain.goTo(0.0, 0.0);
  // Drivetrain.goTo(1000, -1500, true);
  intake.setVelocity(600, rpm);
  intake.spin(fwd);
  Drivetrain.goTo(RedRing);
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

  Drivetrain.calibrate();

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