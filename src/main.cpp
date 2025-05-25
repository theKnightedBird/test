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
#include "robot-config.h"
#include "vantabot.h"

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
// #define MANAGER_ROBOT 1
// Change to redRing if we're red, and blueRing if we're blue
OBJECT allianceRing = RedRing;

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link link(PORT15, "robot_32456_1", linkType::manager);

motor leftMotorA = motor(PORT4, ratio6_1, true);
motor leftMotorB = motor(PORT3, ratio6_1, true);
motor leftMotorC = motor(PORT1, ratio6_1, true);
motor rightMotorA = motor(PORT14, ratio6_1, false);
motor rightMotorB = motor(PORT12, ratio6_1, false);
motor rightMotorC = motor(PORT11, ratio6_1, false);
motor_group leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor_group rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC);
gps GPS = gps(PORT17, 100, 0, distanceUnits::mm, 90);
inertial imu = inertial(PORT6);
vantadrive drive = vantadrive(leftDrive, rightDrive, GPS, imu);

motor intake_motor = motor(PORT19, ratio18_1, false);
motor_group intake_group = motor_group(intake_motor);
optical intake_sensor = optical(PORT16);
intaker intake = intaker(intake_group, intake_sensor);

digital_out clamper = digital_out(Brain.ThreeWirePort.B);
distance clamperSensor = distance(PORT13);

vantabot bot = vantabot(drive, intake, clamper, clamperSensor);

#else
#pragma message("building for the worker")
ai::robot_link link(PORT20, "robot_32456_1", linkType::worker);

motor leftMotorA = motor(PORT1, ratio6_1, true);
motor leftMotorB = motor(PORT2, ratio6_1, true);
motor leftMotorC = motor(PORT19, ratio6_1, true);
motor rightMotorA = motor(PORT10, ratio6_1, false);
motor rightMotorB = motor(PORT11, ratio6_1, false);
motor rightMotorC = motor(PORT20, ratio6_1, false);
motor_group leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor_group rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial imu = inertial(PORT17);
gps GPS = gps(PORT3, -85, 90, distanceUnits::mm, -94.0);
vantadrive drive = vantadrive(leftDrive, rightDrive, GPS, imu);

motor intake_motor_A = motor(PORT8, ratio18_1, true);
motor intake_motor_B = motor(PORT9, ratio18_1, true);
motor_group intake_group = motor_group(intake_motor_A, intake_motor_B);
optical intake_sensor = optical(PORT16);
intaker intake = intaker(intake_group, intake_sensor);

digital_out clamper = digital_out(Brain.ThreeWirePort.A);
distance clamper_sens = distance(PORT5);

vantabot bot = vantabot(drive, intake, clamper, clamper_sens);
#endif

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
  // drive.turnTo(0, 0);
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
  drive.calibrate();
  bot.grabGoal();
  //  bot.findAndScoreRing(allianceRing);

  // drive.driveTo(1200.0, 1200.0, true);
  // drive.driveTo(-1200.0, 1200.0, true);
  // drive.driveTo(-1200.0, -1200.0, true);
  //drive.driveTo(1200.0, -1200.0);

  // drive.drive(50.0, 1000.0);
  // wait(1, sec);
  // drive.drive(50.0, 1000.0, true);

  //     while (true)
  //     {
  //       if (!bot.hasGoal())
  //       {
  //         bot.grabGoal();
  //       }
  //       else if (intake.getNumRingsInGoal() < 6)
  //       {
  //         bot.findAndScoreRing();
  //       }
  //       else
  //       {
  //         bot.scoreInPositiveCorner();
  //       }
  //     }
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

  // drive.calibrate();

  auto_Interaction();
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
  Competition.drivercontrol(autonomousMain);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  FILE *fp = fopen("/dev/serial2", "wb");
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