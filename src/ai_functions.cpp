/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include <string>
#include <iostream>
using namespace vex;
using namespace std;


// // scores a ring
// // TODO: add sensor logic when they put on the sensor
// void score_ring(void)
// {
//     intake.spin(fwd);
//     if (isRed)
//     {
//         goToObject(RedRing, false);
//     }
//     else
//     {
//         goToObject(BlueRing, false);
//     }
// }

// // clear the corner
// void clear_corner(void)
// {
//     if (isRed)
//     {
//         moveToPosition(-170, -170, 180 + 45);
//     }
//     else
//     {
//         moveToPosition(170, 170, 45);
//     }
//     doinker.set(true);
//     Drivetrain.turnFor(90, degrees, true);
//     doinker.set(false);
// }

// // drop alliance in corner
// void drop_in_corner(void)
// {
//     if (isRed)
//     {
//         moveToPosition(-170, -170, 45);
//     }
//     else
//     {
//         moveToPosition(170, 170, 180 + 45);
//     }
//     clear_corner();
//     clamp.set(false);
//     Drivetrain.driveFor(10, distanceUnits::cm, true);
// }

// Function to grab a ring when the arm is positioned over