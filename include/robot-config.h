#pragma once
#include "vantadrive.h"
#include "intaker.h"

using namespace vex;

extern brain Brain;

extern bool isRed;

extern motor_group leftDrive;
extern motor_group rightDrive;
extern gps GPS;
extern vantadrive drive;
extern digital_out clamp;
extern digital_out doinker;
extern intaker intake;
extern controller Controller;

void vexcodeInit(void);
