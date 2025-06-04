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
// The Demo is symmetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define MANAGER_ROBOT 1
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
vex::distance clamperSensor = vex::distance(PORT13);
digital_out doinker = digital_out(Brain.ThreeWirePort.E);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.H);

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
digital_out intakeLift = digital_out(Brain.ThreeWirePort.B)
    vex::distance clamper_sens = vex::distance(PORT5);

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
  intake.intake();
#if defined(MANAGER_ROBOT)
  doinker.set(true);
  drive.driveTo(20, 220, false, 50, false);
  drive.driveTo(1000, 1000, true, 50, false);
  doinker.set(false);
  drive.driveTo(800, 500, true, 50, false);
  clamper.set(true);
  intake.intake();
  intakeLift.set(true);
  drive.driveTo(600, 1400, false, 50, true);
  intakeLift.set(false);
  vexDelay(200);
  drive.driveTo(1400, 1200, false, 50, true);
#else
  drive.driveTo(1150, 100, true);
  vexDelay(100);
  clamper.set(true); // stake
  intake.intake();
  drive.driveTo(1500, 0); // ring 2
  drive.driveTo(1400, 0, false, 30, false);
  drive.driveTo(1500, 700, false, 50, false);
  drive.driveTo(1250, -1300, false, 75, true); // ring 3
  drive.driveTo(100, -1400, false, 50, false);
  vexDelay(200);
  // drive.driveTo(0,-1400,false,50,false);//ring 5
  clamper.set(false);
  intake_motor.spinFor(reverse, 100, msec);
  drive.driveTo(1500, -1550, true, 90, true);
  drive.driveTo(800, -1200, false, 50, false);
#endif
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
  double rings_in_goal = 0;
  bool goal_in_corner = false;
  while (true)
  {
    if (!bot.hasGoal())
    {
      bot.grab_goal();
    }
    else if (rings_in_goal > 5 && goal_in_corner)
    {
      bot.go_to_sector();
      bot.tipOverGoal();
    }
    else if (rings_in_goal > 5)
    {
      bot.score_in_positive_corner();
    }
    else
    {
      bot.find_and_score_ring(allianceRing);
    }
  }
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

  drive.calibrate();

  if (firstAutoFlag)
    auto_Isolation();
  else
    auto_Interaction();

  firstAutoFlag = false;
}

int main()
{

  ostringstream oss;
  oss << vec::dist_between(vec({500, 500}), vec({800, 300}));

  printf("%s\n", oss.str().c_str());
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