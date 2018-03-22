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
#include <Autonomous/Steps/RunIntakeWithDelay.h>
#include <Autonomous/Steps/IntakeSolenoidWithDelay.h>
#include <Autonomous/Steps/IntakeRotate.h>


void DebugAutoStrategy::Init(std::shared_ptr<World> world) {
	std::cout << "DebugAutoStrategy::Init()\n";
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	const double angle = -180;
	SetGyroOffset *step = new SetGyroOffset(angle);
	step->Run(world);
}


DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	const double firstDriveSpeed = 0.5;
	const double firstDriveX = 0.0;
//	const double firstDriveY = 216.0;   36, 186

	const double angle = -180.0;

	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, 0.0001, 0.0, 0.1, false),
		new Delay(0.5),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		new PositionMast(Mast::MastPosition::kDrive),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));


//	steps.push_back(new ConcurrentStep({
//		new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, 36.0, -1, DriveUnit::Units::kInches, 8.0, 0.5, -1),
//		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0),
//		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
//		new PositionMast(Mast::MastPosition::kVertical),
//	}));

	double ydist = 148.0 + 36;
	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, ydist, -1, DriveUnit::Units::kInches, 8.0, 0.5, 24),
		new PositionElevator(Elevator::ElevatorPosition::kHighScale, DelayParam(DelayParam::DelayType::kTime, 1.5)),
		new IntakeRotate(false, 1.5),
		new PositionMast(Mast::MastPosition::kDrive),
	}));


	const double secondDriveSpeed = 0.3;
	const double secondDriveX = -50.0;
	const double secondDriveY = 84.0;

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 6),
		new IntakeRotate(true, 3.0)
	}, true));

	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));

//	SwitchScale(world);
//	ScalePickup(world);
//	CrossField(world);
}

void DebugAutoStrategy::CrossField(std::shared_ptr<World> world) {
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	const double angle = -90.0 * inv;

	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, 0.0001, 0.0, 0.1, false),
		new Delay(1.0),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		new PositionMast(Mast::MastPosition::kVertical),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));

	const double firstDriveSpeed = 0.5;
	const double firstDriveX = 15.0;
	const double firstDriveY = 216.0;

	steps.push_back(new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 30));

	const double secondDriveSpeed = 0.3;
	const double secondDriveX = -90.0;
	const double secondDriveY = 0.0;

	steps.push_back(new ClosedLoopDrive2(angle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 30));


}

void DebugAutoStrategy::ScalePickup(std::shared_ptr<World> world) {
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	const double angle = -90.0 * inv;

	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, 0.0001, 0.0, 0.1, false),
		new Delay(1.0),
		new PositionElevator(Elevator::ElevatorPosition::kLowScale),
		new PositionMast(Mast::MastPosition::kVertical),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));

	const double firstDriveSpeed = 0.5;
	const double firstDriveX = 15.0;
	const double firstDriveY = 313;

	steps.push_back(new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 30));
	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));

	const double secondDriveSpeed = 0.3;
	const double secondDriveX = 0.0;
	const double secondDriveY = -79.0;

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 30),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, true),
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}));


	const double turnAngle = angle - (45 * inv);
	steps.push_back(new Rotate(turnAngle, 10.0, 10.0, 5));

	/************ DOING PICKUP OF SECOND CUBE *****/

	/**
	 * Drive until we pickup or timeout
	 */
	const double drive3Speed = 0.3;
	const double drive3Angle = angle - (45 * inv);
	const double drive3X = -48.0 * inv;
	const double intakeDelayPos = 2 * fabs(drive3X) / 3;

	ClosedLoopDrive2 *drive3 = new ClosedLoopDrive2(drive3Angle, drive3Speed, drive3X, 0.0, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1);
	drive3->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		drive3,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kPosition, intakeDelayPos), 5.0)
	}, true));



	const double drive4Angle = -70;	// was drive3angle
	const double drive4X = 12 * inv;
	const double drive4Y = 90.0;
	ClosedLoopDrive2 *drive4 = new ClosedLoopDrive2(drive4Angle, drive3Speed, drive4X, drive4Y, -1, DriveUnit::Units::kInches, 5.0, 0.5, 12);
	drive4->UsePickupDistance();
	steps.push_back(new ConcurrentStep({
		drive4,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1),
		new PositionElevator(Elevator::ElevatorPosition::kHighScale, DelayParam(DelayParam::DelayType::kTime, 0.5), true)
	}, true));


	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));

}


void DebugAutoStrategy::SwitchScale(std::shared_ptr<World> world) {
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	const double angle = -90.0 * inv;

	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, 0.0001, 0.0, 0.1, false),
		new Delay(1.0),
		new PositionElevator(Elevator::ElevatorPosition::kLowScale),
		new PositionMast(Mast::MastPosition::kVertical)
	}));



	const double firstDriveSpeed = 0.2;
	const double firstDriveX = 0.0;
	const double firstDriveY = 146;
	const double firstEjectY = 142;

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, -1),
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kPosition, firstEjectY), 2.0, -1.0)
	}));


	/*****************************/

	const double secondDriveX = 0.0;
	const double secondDriveY = 75.0;

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, firstDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 4.0, -1, 6),
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1),	// was RunIntake(false, false)
	}));


	// Turn and drop
	const double turnAngle = angle - (45 * inv);
	steps.push_back(new ConcurrentStep({
		new Rotate(turnAngle, 10.0, 10.0, 5),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kTime, 0.5), true)
	}, true));


	/************ DOING PICKUP OF SECOND CUBE *****/

	/**
	 * Drive until we pickup or timeout
	 */
	const double drive3Angle = angle - (45 * inv);
	const double drive3X = -24.0 * inv;
	const double intakeDelayPos = 2 * fabs(drive3X) / 3;

	ClosedLoopDrive2 *drive3 = new ClosedLoopDrive2(drive3Angle, firstDriveSpeed, drive3X, 0.0, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1);
	drive3->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		drive3,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kPosition, intakeDelayPos), 5.0)
	}, true));



	const double drive4Angle = -70;	// was drive3angle
	const double drive4X = 12 * inv;
	const double drive4Y = 90.0;
	ClosedLoopDrive2 *drive4 = new ClosedLoopDrive2(drive4Angle, firstDriveSpeed, drive4X, drive4Y, -1, DriveUnit::Units::kInches, 5.0, 0.5, 12);
	drive4->UsePickupDistance();
	steps.push_back(new ConcurrentStep({
		drive4,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1),
		new PositionElevator(Elevator::ElevatorPosition::kHighScale, DelayParam(DelayParam::DelayType::kTime, 0.5), true)
	}, true));


	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));
}


