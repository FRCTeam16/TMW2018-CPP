/*
 * SideStrategy.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#include <Autonomous/Strategies/SideStrategy.h>
#include <iostream>
#include <Autonomous/World.h>
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/PositionElevator.h>
#include <Autonomous/Steps/PositionMast.h>
#include <Autonomous/Steps/ClosedLoopDrive2.h>
#include <Autonomous/Steps/DriveToBump.h>
#include <Autonomous/Steps/EjectCube.h>
#include <Subsystems/Elevator.h>
#include <Autonomous/Steps/DelayParam.h>

SideStrategy::SideStrategy(std::shared_ptr<World> world) {
	FieldInfo fieldInfo = world->GetFieldInfo();
	AutoStartPosition startPosition = world->GetStartPosition();

	bool haveSwitch;
	bool haveScale;
	if (AutoStartPosition::kCenter == startPosition) {
		std::cerr << "Specified a center start but selected a side strategy, falling back to drive line strategy\n";
	} else if (AutoStartPosition::kLeft == startPosition) {
		haveSwitch = FieldInfo::Left == fieldInfo.switchLocation;
		haveScale = FieldInfo::Left == fieldInfo.scaleLocation;
	} else if (AutoStartPosition::kRight == startPosition) {
		haveSwitch = FieldInfo::Right == fieldInfo.switchLocation;
		haveScale = FieldInfo::Right == fieldInfo.scaleLocation;
	}


	if (haveSwitch && haveScale) {

//		steps.push_back(__x)



	} else if (haveSwitch && !haveScale) {
		steps.push_back(
				new ConcurrentStep({
					new ClosedLoopDrive2(0.0, 0.3, 0, 85, -1, DriveUnit::Units::kInches, 10.0, 0.5, 30000),
					new PositionElevator(Elevator::ElevatorPosition::kSwitch),
					new PositionMast(Mast::MastPosition::kVertical, DelayParam(DelayParam::DelayType::kTime, 0.25)) })
		);

	} else if (!haveSwitch && haveScale) {
		// scale only

	} else {
		// no switch and no scale
	}


}

SideStrategy::~SideStrategy() {
	// TODO Auto-generated destructor stub
}

