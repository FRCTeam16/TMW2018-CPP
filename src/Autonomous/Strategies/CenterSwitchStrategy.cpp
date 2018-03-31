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
#include <Autonomous/Steps/SetGyroOffset.h>
#include <Autonomous/Steps/IntakeSolenoidWithDelay.h>
#include <Autonomous/Steps/RunIntakeWithDelay.h>



CenterSwitchStrategy::CenterSwitchStrategy(std::shared_ptr<World> world) {
	DoTimedDrive(world);
}



void CenterSwitchStrategy::DoTimedDrive(std::shared_ptr<World> world) {
	const AutoStartPosition startPosition = world->GetStartPosition();
	bool isLeft = (FieldInfo::Location::Left == world->GetFieldInfo().switchLocation);
	bool isRight = !isLeft;
	const int inv = isRight ? 1 : -1;


	// Original was 60" X, 108" Y
	// 0.3 y speed, 0.1665 x
	// invtan = 60.94
	// tan (rads) 1.064

	// atan2(x,y) = 0.5
	//

	const double startAngle = 0.0;
	const double speed = PrefUtil::getSet("AutoCenterSpeed", 0.35);
	const double yPos = PrefUtil::getSet("AutoCenterY", 108);
	const double xPos = PrefUtil::getSet("AutoCenterX", 60) * inv;
	const double xOffset = PrefUtil::getSet("AutoCenterXOffset", 24);
	const double collisionThreshold = PrefUtil::getSet("AutoCenterCollisionThreshold", 0.5);

	const double xTotal = xPos - xOffset;
	const double angleRadians = atan2(xTotal, yPos);
	const double xSpeed = speed * sin(angleRadians);
	const double ySpeed = speed * cos(angleRadians);

	std::cout << "isLeft ? " << isLeft << " isRight? " << isRight
			  << " | xPos " << xPos << " inv? " << inv << "\n";

	SetGyroOffset *step = new SetGyroOffset(startAngle);
	step->Run(world);

	const double timeout = 3.5;

	DriveToBump *bumpDrive = new DriveToBump(startAngle, ySpeed, xSpeed, timeout, 1.0, collisionThreshold );
	bumpDrive->SetRampTime(0.5);
	steps.push_back(
			new ConcurrentStep({
					bumpDrive,
					new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0),
					new PositionElevator(Elevator::ElevatorPosition::kSwitch),
					new PositionMast(
							Mast::MastPosition::kVertical,
							DelayParam(DelayParam::DelayType::kTime, 0.25), false),
					new EjectCube(1.0,  timeout,  0.5,  collisionThreshold)
	}));


	const double driveSpeed2 = 0.4;
	const double driveX2 = (-60.0 * inv);
	const double driveY2 = -75.0;

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(startAngle, driveSpeed2, driveX2, driveY2, -1, DriveUnit::Units::kInches, 30.0, 0.5, 6);
	steps.push_back(new ConcurrentStep({
		drive,
	}, true));

	steps.push_back(new ConcurrentStep({
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0)),
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0),
	}));


	//
	// Drive forward and pick up cube
	//
	const double driveSpeed3 = 0.3;
	const double driveX3 = 0.0;
	const double driveY3 = 40.0;

	ClosedLoopDrive2 *drive3 = new ClosedLoopDrive2(startAngle, driveSpeed3, driveX3, driveY3, -1, DriveUnit::Units::kInches, 5.0, 0.5, 6);
	drive3->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		drive3,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
	}, true));


	RunIntakeWithDelay *fiftyIntake1 = new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0);
	fiftyIntake1->SetIntakeSpeed(-0.5);
	steps.push_back(new ConcurrentStep({
		fiftyIntake1,
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));


	//
	// Drive horizontally
	//
	const double driveSpeed4 = 0.4;
	const double driveX4 = 60.0 * inv;
	const double driveY4 = -4.0;
	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(startAngle, driveSpeed4, driveX4, driveY4, -1, DriveUnit::Units::kInches, 5.0, 0.5, 6),
//		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kTime, 0.0), 0.1, 0.0)
	}));


	//
	// 6. Drive forward and score
	//
	const double driveSpeed5 = 0.3;
//	const double driveX5 = 0.0;
//	const double driveY5 = 45.0;
	const double drive5Timeout = 2.0;
//	ClosedLoopDrive2 *drive5 = new ClosedLoopDrive2(startAngle, driveSpeed5, driveX5, driveY5, -1, DriveUnit::Units::kInches, 5.0, 0.5, 6);
//	drive5->SetHardStopsContinueFromStep(true);

	DriveToBump *drive5 = new DriveToBump(startAngle, 0.4, 0, timeout, 2.0, collisionThreshold );
	steps.push_back(new ConcurrentStep({
		drive5,
		new PositionElevator(Elevator::ElevatorPosition::kSwitch, DelayParam(DelayParam::DelayType::kNone, 0.0)),
//		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kTime, drive5Timeout), 5.0, 0.5)
		new EjectCube(1.0,  drive5Timeout,  0.5,  collisionThreshold)
	}, true));


	//
	// Back to pile
	//
	const double driveSpeed6 = 0.4;
	const double driveX6 = (-60.0 * inv);
	const double driveY6 = -75.0;
	const int drive6Elevator = 5500;

	ClosedLoopDrive2 *drive6 = new ClosedLoopDrive2(startAngle, driveSpeed6, driveX6, driveY6, -1, DriveUnit::Units::kInches, 30.0, 0.5, 6);
	steps.push_back(new ConcurrentStep({
		drive6,
	}, true));

	PositionElevator *elevator6 = new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0));
	elevator6->SetOverrideElevatorPosition(drive6Elevator);
	steps.push_back(new ConcurrentStep({
		elevator6,
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0),
	}));

	//
	// Drive forward and pick up cube
	//
	const double driveSpeed7 = 0.3;
	const double driveX7 = 0.0;
	const double driveY7 = 52.0;

	ClosedLoopDrive2 *drive7 = new ClosedLoopDrive2(startAngle, driveSpeed7, driveX7, driveY7, -1, DriveUnit::Units::kInches, 5.0, 0.5, 6);
	drive7->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		drive7,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
	}, true));

	steps.push_back(new ConcurrentStep({
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));
}
