
#include <Autonomous/Strategies/SideStrategy.h>
#include <iostream>
#include <Autonomous/Steps/ClosedLoopDrive2.h>
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/Delay.h>
#include <Autonomous/Steps/DelayParam.h>
#include <Autonomous/Steps/DriveToBump.h>
#include <Autonomous/Steps/EjectCube.h>
#include <Autonomous/Steps/EjectCubeWithDelay.h>
#include <Autonomous/Steps/IntakeCube.h>
#include <Autonomous/Steps/IntakeSolenoid.h>
#include <Autonomous/Steps/IntakeSolenoidWithDelay.h>
#include <Autonomous/Steps/PositionElevator.h>
#include <Autonomous/Steps/PositionMast.h>
#include <Autonomous/Steps/Rotate.h>
#include <Autonomous/Steps/RotateUntilPast.h>
#include <Autonomous/Steps/RunIntake.h>
#include <Autonomous/Steps/RunIntakeWithDelay.h>
#include <Autonomous/Steps/SetGyroOffset.h>
#include <Autonomous/Steps/TimedDrive.h>
#include <Autonomous/World.h>
#include <Subsystems/Elevator.h>




SideStrategy::SideStrategy(std::shared_ptr<World> world) {
	const FieldInfo fieldInfo = world->GetFieldInfo();
	const AutoStartPosition startPosition = world->GetStartPosition();

	isLeft = AutoStartPosition::kLeft == startPosition;
	isRight = AutoStartPosition::kRight == startPosition;

	inv = isRight ? 1 : -1;
	startAngle = -90.0 * inv;
	std::cout << "Start Angle = " << startAngle << "\n";
	SetGyroOffset *step = new SetGyroOffset(startAngle);
	step->Run(world);


	bool haveSwitch;
	bool haveScale;
	if (AutoStartPosition::kCenter == startPosition) {
		std::cerr << "Specified a center start but selected a side strategy\n";
		// FIXME: Add default drive line strategy
	} else if (AutoStartPosition::kLeft == startPosition) {
		haveSwitch = FieldInfo::Left == fieldInfo.switchLocation;
		haveScale = FieldInfo::Left == fieldInfo.scaleLocation;
	} else if (AutoStartPosition::kRight == startPosition) {
		haveSwitch = FieldInfo::Right == fieldInfo.switchLocation;
		haveScale = FieldInfo::Right == fieldInfo.scaleLocation;
	}

	//
	// Configure steps
	//
	StartInitialPose();


	if (haveSwitch && haveScale) {
		DoSwitchScale();
	} else if (haveSwitch && !haveScale) {
		DoSwitchPickup();
	} else if (!haveSwitch && haveScale) {
		DoScaleScale();
	} else {
		DoTraverse();
	}
}



/**
 * Starts moving the robot from its initial starting to position to an operational configuration.
 * Allows minimization of structural forces which affect drive accuracy.
 */
void SideStrategy::StartInitialPose() {
	const double poseDelay = PrefUtil::getSet("AutoSideInitialPoseDelay", 4.0);


	steps.push_back(new ConcurrentStep({
		new TimedDrive(startAngle, 0.0001, 0.0, poseDelay, false),
		new Delay(poseDelay),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		new PositionMast(Mast::MastPosition::kVertical),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));
}


/**
 * When we don't have the switch or scale, move to the other side
 */
void SideStrategy::DoTraverse() {
	const double firstDriveSpeed = PrefUtil::getSet("AutoSideTraverseSpeed1", 0.5);
	const double firstDriveX = PrefUtil::getSet("AutoSideTraverseX1", 15.0);
	const double firstDriveY = PrefUtil::getSet("AutoSideTraverseY1", 228.0);

	steps.push_back(new ClosedLoopDrive2(startAngle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 30));

	const double secondDriveSpeed = PrefUtil::getSet("AutoSideTraverseSpeed2", 0.3);
	const double secondDriveX = PrefUtil::getSet("AutoSideTraverseX2", -114.0) * inv;
	const double secondDriveY = PrefUtil::getSet("AutoSideTraverseY2", 0.0);

	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kHighScale));
	steps.push_back(new ClosedLoopDrive2(startAngle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 12));


	const double rotateAngle = PrefUtil::getSet("AutoSideTraverseRotate", -180.0);
	const double driveInX = PrefUtil::getSet("AutoSideTraverseDriveInX", -1);
	const double driveInY = PrefUtil::getSet("AutoSideTraverseDriveInY", -1);
	const double driveInTime = PrefUtil::getSet("AutoSideTraverseDriveInTime", 1.0);

	steps.push_back(new Rotate(rotateAngle, 5, 10.0, 1));
	steps.push_back(new ConcurrentStep({
		new TimedDrive(rotateAngle, driveInY, driveInX, driveInTime, false),
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}));
	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));
}


/**
 * When we have both switch and scale, score scale, pickup, score scale
 */
void SideStrategy::DoSwitchScale() {
	DoSwitchPickup();
	DoSecondCubeScale();
}


void SideStrategy::DoSwitchPickup() {
	//
	// Crab to switch, start eject and drive through for some distance
	//
	const double firstDriveSpeed = PrefUtil::getSet("AutoSideSwitchSpeed1", 0.2);
	const double firstDriveX = PrefUtil::getSet("AutoSideSwitchX1", 0.0) * inv;
	const double firstDriveY = PrefUtil::getSet("AutoSideSwitchY1", 146);
	const double firstEjectY = PrefUtil::getSet("AutoSideSwitchEjectY1", 142);
	const double firstRampUp = PrefUtil::getSet("AutoSideSwitchRampUp1", 0.5);

	ClosedLoopDrive2 *step = new ClosedLoopDrive2(startAngle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 5.0, firstRampUp, -1);
	step->SetHardStopsContinueFromStep(false);
	steps.push_back(new ConcurrentStep({
		step,
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0),
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kPosition, firstEjectY), 2.0, -1)
	}));


	//
	// Drive to location to prepare for second cube pickup
	//
	const double secondDriveX = PrefUtil::getSet("AutoSideSwitchX2", 0.0) * inv;
	const double secondDriveY = PrefUtil::getSet("AutoSideSwitchY2", 75.0);
	const double secondRampDown = PrefUtil::getSet("AutoSideSwitchRampDown2", 6.0);

	ClosedLoopDrive2 *step2 = new ClosedLoopDrive2(startAngle, firstDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 4.0, -1, secondRampDown);
	step2->SetHardStopsContinueFromStep(false);
	steps.push_back(new ConcurrentStep({
		step2,
	}));


	// Turn and drop elevator
	const double rotateAngleOffset = PrefUtil::getSet("AutoSideSwitchRotateAngle", 45.0) * inv;
	const double turnAngle = startAngle - rotateAngleOffset;
	const double rotateAngleThreshold = PrefUtil::getSet("AutoSideSwitchRotateAngleThreshold", 10.0);
	const int rotateAngleThresholdScans = PrefUtil::getSet("AutoSideSwitchRotateAngleThresholdScans", 5);

	steps.push_back(new ConcurrentStep({
		new Rotate(turnAngle, rotateAngleThreshold, 10.0, rotateAngleThresholdScans),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kTime, 0.5), true),
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1),
	}, true));


	/************ DOING PICKUP OF SECOND CUBE *****/
	const double xPickupDistance = PrefUtil::getSet("AutoSideSwitchPickupX", -24.0) * inv;
	DoSecondCubePickup(turnAngle, xPickupDistance);
}

/**
* When we have only scale
* */
void SideStrategy::DoScaleScale() {

	const double firstDriveSpeed = PrefUtil::getSet("AutoSideScaleSpeed1", 0.5);
	const double firstDriveX = PrefUtil::getSet("AutoSideScaleX1", 15.0) * inv;
	const double firstDriveY = PrefUtil::getSet("AutoSideScaleY1", 313);

	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kHighScale));
	steps.push_back(new ClosedLoopDrive2(startAngle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 30.0, 0.5, 30));
	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));


	const double secondDriveSpeed = PrefUtil::getSet("AutoSideScaleSpeed2", 0.3);
	const double secondDriveX = PrefUtil::getSet("AutoSideScaleX2", 0.0) * inv;
	const double secondDriveY = PrefUtil::getSet("AutoSideScaleY2", -79.0);
	const double secondDriveRampUp = PrefUtil::getSet("AutoSideScaleRampUp", 0.5);
	const double secondDriveRampDown = PrefUtil::getSet("AutoSideScaleRampDown", 10);

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(startAngle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 8.0, secondDriveRampUp, secondDriveRampDown),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, true),
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}));


	const double rotateAngleOffset = PrefUtil::getSet("AutoSideScaleRotateAngle", 45.0) * inv;
	const double turnAngle = startAngle - rotateAngleOffset;
	const double rotateAngleThreshold = PrefUtil::getSet("AutoSideScaleRotateAngleThreshold", 10.0);
	const int rotateAngleThresholdScans = PrefUtil::getSet("AutoSideScaleRotateAngleThresholdScans", 5);

	steps.push_back(new Rotate(turnAngle, rotateAngleThreshold, 10.0, rotateAngleThresholdScans));

	/************ DOING PICKUP OF SECOND CUBE *****/
	const double xPickupDistance = PrefUtil::getSet("AutoSideScalePickupX", -48.0) * inv;
	DoSecondCubePickup(turnAngle, xPickupDistance);
	DoSecondCubeScale();
}

/**
 * Drives into cube until distance met or auto pickup threshold detected
 */
void SideStrategy::DoSecondCubePickup(double robotAngle, double xDriveDistance) {
	const double speed = PrefUtil::getSet("AutoSidePickupSpeed", 0.2);
	const double driveY = PrefUtil::getSet("AutoSidePickupY", 0.0);
	const double intakeDelayPos = 2 * fabs(xDriveDistance) / 3;	// delay until closing the intake solenoid to trap the cube

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(robotAngle, speed, xDriveDistance, driveY, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1);
	drive->SetHaltOnIntakePickup(true);
	drive->SetHardStopsContinueFromStep(false);
	steps.push_back(new ConcurrentStep({
		drive,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kPosition, intakeDelayPos), 5.0)
	}, true));
}

/**
 * Drives back to known point plus variable offset distance from previous drive step required to pickup
 */
void SideStrategy::DoSecondCubeScale() {
	const double driveAngle = PrefUtil::getSet("AutoSidePickupScaleAngle", -70) * inv;
	const double speed = PrefUtil::getSet("AutoSidePickupScaleSpeed", 0.2);
	const double X = PrefUtil::getSet("AutoSidePickupScaleX", 12) * inv;
	const double Y = PrefUtil::getSet("AutoSidePickupScaleY", 90.0);

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(driveAngle, speed, X, Y, -1, DriveUnit::Units::kInches, 5.0, 0.5, -1);
	drive->UsePickupDistance(isLeft);	// use pickup distance as additional x offset, inverting the direction if we are on the left side
	steps.push_back(new ConcurrentStep({
		drive,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1),
		new PositionElevator(Elevator::ElevatorPosition::kHighScale, DelayParam(DelayParam::DelayType::kTime, 0.5), true)
	}, true));


	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));
}
