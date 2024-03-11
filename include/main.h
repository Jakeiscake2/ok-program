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

#endif  // _PROS_MAIN_H_
