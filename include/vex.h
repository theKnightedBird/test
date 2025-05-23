/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#pragma once
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "v5.h"
#include "v5_vcs.h"
#include <cmath>
#include "ai_jetson.h"
#include "ai_robot_link.h"

using namespace vex;

extern brain Brain;
extern controller Controller;
extern ai::jetson jetson_comms;
extern ai::robot_link link;
extern gps GPS;
extern int marker;

#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)

extern int dashboardTask(void);
