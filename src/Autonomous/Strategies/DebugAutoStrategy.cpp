/*
 * DebugAutoStrategy.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#include "DebugAutoStrategy.h"
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/Delay.h>
#include <Autonomous/Steps/SetGyroOffset.h>
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/PositionElevator.h>
#include <Autonomous/Steps/PositionMast.h>
#include <Autonomous/Steps/ClosedLoopDrive2.h>
#include <Autonomous/Steps/DriveToBump.h>
#include <Autonomous/Steps/EjectCube.h>



DebugAutoStrategy::DebugAutoStrategy() {

	steps.push_back(
			new ConcurrentStep({
				new ClosedLoopDrive2(0.0, 0.3, -13, 110.25, -1, DriveUnit::Units::kInches, 10.0, 0.5, 30000),
		        new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		        new PositionMast(Mast::MastPosition::kVertical, DelayParam(DelayParam::DelayType::kTime, 0.25)),
		        new PositionElevator(
		        		Elevator::ElevatorPosition::kHighScale,
		        		DelayParam(DelayParam::DelayType::kPosition, 100000))
	}));

//	steps.push_back(new StopStep());
}



