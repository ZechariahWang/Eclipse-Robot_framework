/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// Modules
#include "210Z-Lib/Modules/MathModule.hpp"
#include "210Z-Lib/Modules/UtilityModule.hpp"
#include "210Z-Lib/Modules/MetricsReturnModule.hpp"
#include "210Z-Lib/Modules/AutonomousSelectorModule.hpp"
#include "210Z-Lib/Modules/PathTrackingAlgorithmsModule.hpp"
#include "210Z-Lib/Modules/LinearKalmanFilterModule.hpp"
#include "210Z-Lib/Modules/PIDcontrollerModule.hpp"
#include "210Z-Lib/Modules/LinearMotionProfileModule.hpp"
#include "210Z-Lib/Modules/BezierCurvePathGenModule.hpp"
#include "210Z-Lib/Modules/ConstantsTuner.hpp"

// Driver
#include "210Z-Lib/OperationSystems/CoreOpSystems.hpp"

// Chassis
#include "210Z-Lib/Chassis/FeedbackController.hpp"
#include "210Z-Lib/Chassis/AbsolutePositioningSystemController.hpp"
#include "210Z-Lib/Chassis/LateralChassisController.hpp"

// Scripts
#include "Scripts/AutonSelectorFile.hpp"
#include "Scripts/WorldsPaths.hpp"

// lvgl Display
#include "pros/apix.h"
#include "display/lvgl.h"
#include "210Z-Lib/Chassis/InitializeUI.hpp"

// Misc
#pragma once
#include "Config/Globals.hpp"
#include "Config/AssetDirectoryConfig.hpp"
#include "SpecifyGameOpLogic/SecondaryOpSystems.hpp"
#include "SpecifyGameOpLogic/SecondaryOpDriver.hpp"

/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
