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
#include <Autonomous/Steps/TimedDrive.h>


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
	    DoScaleFirstSecondSwitch();
	} else if (haveSwitch && !haveScale) {
		DoSwitchFirst();
	} else if (!haveSwitch && haveScale) {
		DoScaleFirst();
		DoScaleFirstSecondPickup();
		DoScaleFirstSecondScale();
	} else {
		// no switch and no scale
		// TODO: cross the line or try traverse
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
	const int inv = isRight ? 1 : -1;
	const double firstAngleTarget = 180.0 * inv;
	const double firstAngleThreshold = 90.0 * inv;

	const double autoScalePickupSpeed = 0.3;
	const double autoScalePickupY = -48;
	const double autoScalePickupX = -14 * inv;

	steps.push_back(new RotateUntilPast(isRight, firstAngleTarget, firstAngleThreshold));
	steps.push_back(
			new ConcurrentStep({
				new Rotate(firstAngleTarget, 5.0, 5.0, 10),
				new PositionElevator(Elevator::ElevatorPosition::kFloor, true),
				new IntakeSolenoid(true)
	}));
	steps.push_back(
				new ConcurrentStep({
					new ClosedLoopDrive2(firstAngleTarget, autoScalePickupSpeed, autoScalePickupX, autoScalePickupY, -1, DriveUnit::Units::kInches, 5.0, -1, -1),
					new IntakeCube(3.0, 3.5, 4.0)
	}));
}

void SideStrategy::DoScaleFirstSecondScale() {
	const bool isRight = !isLeft;
	const int inv = isRight ? 1 : -1;
	const double firstAngleTarget = -15.0 * inv;

	const double autoScalePickupSpeed = 0.3;
	const double autoScalePickupY = 48;
	const double autoScalePickupX = 14 * inv;


	steps.push_back(new ConcurrentStep({
		new TimedDrive(-180.0, 0.2, 0.0, 0.2),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch)
	}));
	steps.push_back(
			new ConcurrentStep({
				new Rotate(firstAngleTarget, 5.0, 5.0, 10),
				new PositionElevator(Elevator::ElevatorPosition::kHighScale, true),
	}));
	steps.push_back(
		new ClosedLoopDrive2(firstAngleTarget, autoScalePickupSpeed, autoScalePickupX, autoScalePickupY, -1, DriveUnit::Units::kInches, 5.0, 0.5, 24)
	);
	steps.push_back(new EjectCube(0.0, 5.0, 1.0));
}

void SideStrategy::DoScaleFirstSecondSwitch() {
	const bool isRight = !isLeft;
	const int inv = isRight ? 1 : -1;
	const double angle = -165.0 * inv;

	const double driveInSpeed = -0.2;
	const double timeToDrive = 1.5;


	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, 0.2, 0.0, 0.5),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch, true)
	}, true));
	steps.push_back(new TimedDrive(angle, driveInSpeed, 0.0, timeToDrive));
	steps.push_back(new EjectCube(0.0, 5.0, 1.0));
}


void SideStrategy::DoSwitchFirst() {
	const double angle = isLeft ? 90.0 : -90.0;
	const double ySpeed1 = 0.4;
	const double xSpeed1 = 0.0;
	const double timeToDrive1 = PrefUtil::getSet("AutoSwitchFirstDriveTime1", 3.0);

	const double ySpeed2 = 0.0;
	const double xSpeed2 = isLeft ? 0.3 : -0.3;
	const double collisionThreshold = 0.5;


	steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, 0.2, 0.0, 0.5),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		new PositionMast(Mast::MastPosition::kVertical)
	}));
	// Need to travel 172 inches if we switch to position drive
	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, ySpeed1, xSpeed1, timeToDrive1)
	}));
	steps.push_back(new ConcurrentStep({
		new DriveToBump(angle, ySpeed2, xSpeed2, 4, 1.0, collisionThreshold ),
		new EjectCube(1.0,  5.0,  0.5,  collisionThreshold)
	}));
}
