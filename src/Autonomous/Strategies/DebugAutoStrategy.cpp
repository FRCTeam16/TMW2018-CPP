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
#include <Autonomous/Steps/TimedDrive.h>
#include <Autonomous/Steps/SetGyroOffset.h>
#include <Autonomous/Steps/EjectCubeWithDelay.h>
#include <Autonomous/Steps/RunIntake.h>



DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	const double angle = -90.0 * inv;

	const double firstDriveSpeed = 0.2;
	const double firstDriveX = 0.0;
	const double firstDriveY = 146;
	const double firstEjectY = 142;


	const double secondDriveX = 20.0 * inv;
	const double secondDriveY = 70.0;

	steps.push_back(new SetGyroOffset(angle));

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, -1),
		new PositionMast(Mast::MastPosition::kVertical,	DelayParam(DelayParam::DelayType::kTime, 0.25), false),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		new EjectCubeWithDelay(DelayParam(DelayParam::DelayType::kPosition, firstEjectY), 2.0, -1.0)
	}));


	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, firstDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 4.0, -1, 24),
		new RunIntake(false, false),
		new PositionElevator(Elevator::ElevatorPosition::kFloor)
	}));


	const double drive3Angle = angle - (45 * inv);
	const double drive3X = -36.0 * inv;

	ClosedLoopDrive2 *drive3 = new ClosedLoopDrive2(drive3Angle, firstDriveSpeed, drive3X, 0.0, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1);
	drive3->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		drive3,
		new RunIntake(true, false)
	}));


	steps.push_back(new ConcurrentStep({
		new IntakeSolenoid(false)
	}, true));

	// todo drive forward to pickup
}

/**
 *
	const int inv = isRight ? 1 : -1;
	const double firstAngleTarget = -15.0 * inv;
	const double firstAngleThreshold = -90.0 * inv;

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
 */

/**
 * steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kHighScale, true));
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

	steps.push_back(new StopStep());
 */



