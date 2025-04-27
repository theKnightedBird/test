using namespace vex;

extern brain Brain;

extern bool isRed;

extern motor leftDrive;
extern motor rightDrive;
extern gps GPS;
extern smartdrive Drivetrain;
extern digital_out clamp;
extern digital_out doinker;
extern motor Arm;
extern motor intake;
extern controller Controller;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
