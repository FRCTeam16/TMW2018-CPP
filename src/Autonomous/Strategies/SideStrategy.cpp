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
#include <Autonomous/Steps/Delay.h>
#include <Autonomous/Steps/Rotate.h>
#include <Autonomous/Steps/RotateUntilPast.h>
#include <Autonomous/Steps/IntakeCube.h>
#include <Autonomous/Steps/IntakeSolenoid.h>

SideStrategy::SideStrategy(std::shared_ptr<World> world) {
	FieldInfo fieldInfo = world->GetFieldInfo();
	AutoStartPosition startPosition = world->GetStartPosition();

	isLeft = AutoStartPosition::kLeft == startPosition;

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
		DoScaleFirst();
		DoScaleFirstSecondPickup();
	} else if (haveSwitch && !haveScale) {

	} else if (!haveSwitch && haveScale) {
		DoScaleFirst();
		DoScaleFirstSecondPickup();
	} else {
		// no switch and no scale
	}

}

void SideStrategy::DoScaleFirst() {
	const double inv = (isLeft) ? -1.0 : 1.0;
	const double scaleSpeed1 = PrefUtil::getSet("AutoScaleSpeed1", 0.3);
	const double scaleSpeed2 = PrefUtil::getSet("AutoScaleSpeed2", 0.3);

	const double dartOffset = (isLeft) ?
			PrefUtil::getSet("AutoScaleX1OffsetL", 0.0) :
			PrefUtil::getSet("AutoScaleX1OffsetR", 0.0);

	const double scaleY1 = PrefUtil::getSet("AutoScaleY1", 160);
	const double scaleX1 = PrefUtil::getSet("AutoScaleX1", -15) + dartOffset;

	const double scaleY2 = PrefUtil::getSet("AutoScaleY2", 91);
	const double scaleX2 = PrefUtil::getSet("AutoScaleX2", -35);


	steps.push_back(
			new ConcurrentStep({
				new Delay(0.3),
				new PositionElevator(Elevator::ElevatorPosition::kSwitch),
				new PositionMast(Mast::MastPosition::kVertical)
			})
	);
	steps.push_back(new ClosedLoopDrive2(0.0, scaleSpeed1, scaleX1 * inv, scaleY1, -1, DriveUnit::Units::kInches, 10.0, 0.5, -1));
	steps.push_back(
			new ConcurrentStep({
					new ClosedLoopDrive2(0.0, scaleSpeed2, scaleX2 * inv, scaleY2, -1, DriveUnit::Units::kInches, 5.0, -1, 33.33),
					new PositionElevator(Elevator::ElevatorPosition::kHighScale)
			})
	);
	steps.push_back(new EjectCube(0.0,  2.0,  0.5));
}

void SideStrategy::DoScaleFirstSecondPickup() {
	const bool isRight = !isLeft;

	steps.push_back(new RotateUntilPast(isRight, 180.0, 90.0));
	steps.push_back(
			new ConcurrentStep({
				new Rotate(180.0, 5.0, 5.0, 10),
				new PositionElevator(Elevator::ElevatorPosition::kFloor, true),
				new IntakeSolenoid(true)
	}));

	steps.push_back(
				new ConcurrentStep({
					new ClosedLoopDrive2(180.0, 0.3, -10, -48, -1, DriveUnit::Units::kInches, 5.0, -1, -1),
					new IntakeCube(3.0, 3.5, 4.0)
	}));
}

