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
#include <Autonomous/Steps/Rotate.h>


DebugAutoStrategy::DebugAutoStrategy() {
	/*
	steps.push_back(
			new ConcurrentStep({
				new ClosedLoopDrive2(0.0, 0.3, -13, 110.25, -1, DriveUnit::Units::kInches),
		        new PositionElevatorStep(Elevator.ElevatorPosition.Switch),
		        new PositionMastStep(Mast.MastPosition.Vertical, 0.25),
		        new PositionElevatorStep(Elevator.ElevatorPosition.HighScale, new DelayParam(DelayParam.DelayType.Position, 100000)) }));
	steps.push_back(new StopStep());
	*/
}



