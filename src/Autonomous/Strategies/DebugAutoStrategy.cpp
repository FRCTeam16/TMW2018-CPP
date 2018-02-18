/*
 * DebugAutoStrategy.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#include "DebugAutoStrategy.h"
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/Delay.h>
#include <Autonomous/Steps/DriveSteps.h>
#include <Autonomous/Steps/SetGyroOffset.h>
#include <Autonomous/Steps/Rotate.h>


DebugAutoStrategy::DebugAutoStrategy() {
	steps.push_back(
			new ConcurrentStep({
					new Delay(0.5),
					new Delay(0.75) }));

}



