/*
 * CenterSwitchStrategy.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#include <Autonomous/Strategies/CenterSwitchStrategy.h>
#include <Autonomous/World.h>
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/PositionElevator.h>
#include <Autonomous/Steps/PositionMast.h>
#include <Autonomous/Steps/ClosedLoopDrive2.h>
#include <Autonomous/Steps/DriveToBump.h>
#include <Autonomous/Steps/EjectCube.h>
#include <Subsystems/Elevator.h>
#include <Autonomous/Steps/DelayParam.h>

CenterSwitchStrategy::CenterSwitchStrategy(std::shared_ptr<World> world) {
	int inv = 1;
	if (FieldInfo::Location::Left == world->GetFieldInfo().switchLocation) {
		inv = -1;
	}

	steps.push_back(
				new ConcurrentStep({
						new DriveToBump(0.0, 0.3, 0.1665 * inv, 4, 0.25, 1.0 ),
						new PositionElevator(Elevator::ElevatorPosition::kSwitch),
						new PositionMast(
								Mast::MastPosition::kVertical,
								DelayParam(DelayParam::DelayType::kTime, 0.25)),
						new EjectCube(1.0,  5.0,  0.5,  0.75)
		}));

//	steps.push_back(
//			new ConcurrentStep({
//					new ClosedLoopDrive2(0.0, 0.5, 60 * inv, 108, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1),
//					new PositionElevator(Elevator::ElevatorPosition::kSwitch),
//					new PositionMast(
//							Mast::MastPosition::kVertical,
//							DelayParam(DelayParam::DelayType::kTime, 0.25)),
//					new EjectCube(1.0,  4.0,  0.5,  1.0)
//	}));
//	steps.push_back(new DriveToBump(0.0, 0.15, 0.0, 1.5, 0.1, 1.0));
//	steps.push_back(new EjectCube(0.0, 3.0, 0.25));

}

CenterSwitchStrategy::~CenterSwitchStrategy() {
	// TODO Auto-generated destructor stub
}

