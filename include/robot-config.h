#pragma once
#include "vantadrive.h"

using namespace vex;

extern brain Brain;

extern bool isRed;

extern motor_group leftDrive;
extern motor_group rightDrive;
extern gps GPS;
extern vantadrive Drivetrain;
extern digital_out clamp;
extern digital_out doinker;
extern motor intake;
extern controller Controller;

void vexcodeInit(void);
