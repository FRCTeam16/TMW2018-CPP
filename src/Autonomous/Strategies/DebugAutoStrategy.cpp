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
#include <Autonomous/Steps/Rotate.h>
#include <Autonomous/Steps/RotateUntilPast.h>
#include <Autonomous/Steps/IntakeSolenoid.h>
#include <Autonomous/Steps/IntakeCube.h>



DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	AutoStartPosition startPosition = world->GetStartPosition();
	bool isRight = AutoStartPosition::kRight == startPosition;

	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kHighScale, true));
	steps.push_back(new Delay(2.0));

	steps.push_back(new RotateUntilPast(isRight, 180.0, 90.0));
	steps.push_back(
			new ConcurrentStep({
				new Rotate(180.0, 5.0, 5.0, 10),
		        new PositionElevator(Elevator::ElevatorPosition::kFloor, true),
				new IntakeSolenoid(true)
	}));

	steps.push_back(
				new ConcurrentStep({
					new ClosedLoopDrive2(180.0, 0.2, -6, -12, -1, DriveUnit::Units::kInches, 3.0, -1, -1),
					new IntakeCube(3.0, 3.5, 4.0)
	}));

//	steps.push_back(new StopStep());
}



