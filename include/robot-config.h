#pragma once
#include "vex.h"
#include "intake.hpp"
#include "arm.hpp"
#include "clamp.hpp"
#include "vbdrive.hpp"
#include "ai_robot_link.h"

using namespace vex;

extern VB_Drive drive;
extern Arm arm;
extern Clamp clamp;
extern Intake intake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
