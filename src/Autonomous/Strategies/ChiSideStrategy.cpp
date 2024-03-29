
#include <Autonomous/Strategies/ChiSideStrategy.h>
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
#include <Util/DistanceControl.h>


ChiSideStrategy::ChiSideStrategy(std::shared_ptr<World> world) {
	const FieldInfo fieldInfo = world->GetFieldInfo();
	const AutoStartPosition startPosition = world->GetStartPosition();
	const bool traverse = world->GetAutoTraverse();

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
		std::cout << "!!! WARNING - Selected a center start position with a side-based strategy !!!\n";
	} else if (AutoStartPosition::kLeft == startPosition) {
		haveSwitch = FieldInfo::Left == fieldInfo.switchLocation;
		haveScale = FieldInfo::Left == fieldInfo.scaleLocation;
	} else if (AutoStartPosition::kRight == startPosition) {
		haveSwitch = FieldInfo::Right == fieldInfo.switchLocation;
		haveScale = FieldInfo::Right == fieldInfo.scaleLocation;
	}

	std::cout << "isLeft? " << isLeft
			<< " | haveSwitch? " << haveSwitch << " | haveScale? " << haveScale
			<< " | traverse? " << traverse << "\n";


	//------------------------------------------------------------------------
	// Configure steps
	//------------------------------------------------------------------------


	if (haveScale) {
		std::cout << "[Auto] Doing Scale\n";
		StartInitialPose(Mast::MastPosition::kDrive);
		DoFirstScale();
		DoSecondScale();
	} else {
		StartInitialPose();
		if (traverse) {
			std::cout << "[Auto] Doing Traverse\n";
			DoTraverse();
		} else {
			if (haveSwitch) {
				std::cout << "[Auto] Doing Switch\n";
				DoSwitchPickup();
			} else {
				std::cout << "[Auto] Crossing Line\n";
				const bool doNearTraverse = Preferences::GetInstance()->GetBoolean("AutoCrossNearSideTraverse", false);
				Preferences::GetInstance()->PutBoolean("AutoCrossNearSideTraverse", doNearTraverse);
				std::cout << "\tNear Side Traverse? " << doNearTraverse << "\n";
				if (doNearTraverse) {
					CrossLineNearTraverse();
				} else {
					CrossLine();
				}
			}
		}
	}
}



/**
 * Starts moving the robot from its initial starting to position to an operational configuration.
 * Allows minimization of structural forces which affect drive accuracy.
 */
void ChiSideStrategy::StartInitialPose(Mast::MastPosition initialPosition) {
	const double poseDelay = PrefUtil::getSet("AutoSideInitialPoseDelay", 1.0);

	steps.push_back(new ConcurrentStep({
		new TimedDrive(startAngle, 0.0001, 0.0, poseDelay, false),
		new Delay(poseDelay),
		new PositionElevator(Elevator::ElevatorPosition::kSwitch),
		new PositionMast(initialPosition),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}));
}


/**
 * When we don't have the switch or scale, move to the other side
 */
void ChiSideStrategy::DoTraverse() {
	const double firstDriveSpeed = PrefUtil::getSet("AutoSideTraverseSpeed1", 0.5);
	const double firstDriveX = PrefUtil::getSet("AutoSideTraverseX1", 15.0) * inv;
	const double firstDriveY = PrefUtil::getSet("AutoSideTraverseY1", 228.0);
	const double distCtrlX = PrefUtil::getSet("AutoSideLongDistCtrlX", 0.25);

	ClosedLoopDrive2 *firstDrive = new ClosedLoopDrive2(startAngle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 4.5, 1.5, 30);
	firstDrive->EnableDistanceControl(distCtrlX, isRight);
	steps.push_back(firstDrive);

	//
	// Traverse the field horizontally
	//

	const double secondDriveSpeed = PrefUtil::getSet("AutoSideTraverseSpeed2", 0.3);
	const double secondDriveX = PrefUtil::getSet("AutoSideTraverseX2", -114.0) * inv;
	const double secondDriveY = PrefUtil::getSet("AutoSideTraverseY2", 0.0);

	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kHighScale));
	steps.push_back(new ClosedLoopDrive2(startAngle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, 24));

	//
	// Rotate, drive to scale at an angle, eject
	//

	const double rotateAngle = PrefUtil::getSet("AutoSideTraverseRotate", 20.0) * inv;
	const double driveInSpeed = PrefUtil::getSet("AutoSideTraverseDriveInSpeed", 0.3);
	const double driveInX = PrefUtil::getSet("AutoSideTraverseDriveInX", 0);
	const double driveInY = PrefUtil::getSet("AutoSideTraverseDriveInY", 36.0);
//	const double driveInTime = PrefUtil::getSet("AutoSideTraverseDriveInTime", 1.0);

	Rotate *rotateToScale = new Rotate(rotateAngle, 5, 2.0, 1);
	rotateToScale->SetContinueOnTimeout(true);
	steps.push_back(rotateToScale);
	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(rotateAngle, driveInSpeed, driveInX, driveInY, -1, DriveUnit::Units::kInches, 3.0, 0.25, 5)
//		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}, true));
	RunIntakeWithDelay *eject = new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, 1.0);
	eject->SetEjectSpeed(0.50);
	steps.push_back(eject);


	//
	// Get setup for pickup
	//
/*
	const double pickupAngle = PrefUtil::getSet("AutoSideTraversePickupRotate", 180.0) * inv;
	const double pickupAngleThreshold = PrefUtil::getSet("AutoSideTraversePickupRotateThreshold", 10.0) * inv;
	const double pickupAngleScans = PrefUtil::getSet("AutoSideTraversePickupRotateThresholdScans", 5) * inv;
	const double pickupSpeed = PrefUtil::getSet("AutoSideTraversePickupSpeed", 0.3);
	const double pickupX = PrefUtil::getSet("AutoSideTraversePickupX", 22) * inv;
	const double pickupY = PrefUtil::getSet("AutoSideTraversePickupY", 36);

	steps.push_back(new ClosedLoopDrive2(rotateAngle, 0.3, 0, -12, -1, DriveUnit::Units::kInches, 1.0, 0.25, 4));
	Rotate *transversePickupRotate = new Rotate(pickupAngle, pickupAngleThreshold, 3.0, pickupAngleScans);
	transversePickupRotate->SetContinueOnTimeout(true);
	steps.push_back(new ConcurrentStep({
		transversePickupRotate,
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kTime, 0.75), true),
	}));


	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(pickupAngle, pickupSpeed, pickupX, pickupY, -1, DriveUnit::Units::kInches, 4.0, 0.25, 5);
	drive->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		drive,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kPosition, 12.0), 1.0),
	}));*/

}


/**
 * When we have both switch and scale, score scale, pickup, score scale
 */
void ChiSideStrategy::DoSwitchScale() {
	DoSwitchPickup();
	DoSecondCubeScale();
}


void ChiSideStrategy::DoSwitchPickup() {
	//
	// Crab to switch, start eject and drive through for some distance
	//
	const double firstDriveSpeed = PrefUtil::getSet("AutoSideSwitchSpeed1", 0.2);
	const double firstDriveX = PrefUtil::getSet("AutoSideSwitchX1", 0.0) * inv;
	const double firstDriveY = PrefUtil::getSet("AutoSideSwitchY1", 146);
	const double firstEjectY = PrefUtil::getSet("AutoSideSwitchEjectY1", 142);
	const double firstRampUp = PrefUtil::getSet("AutoSideSwitchRampUp1", 0.5);
	const double firstXDist = PrefUtil::getSet("AutoSideSwitchDistCtrlX", 0.42);

	ClosedLoopDrive2 *step = new ClosedLoopDrive2(startAngle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 5.0, firstRampUp, -1);
	step->SetHardStopsContinueFromStep(false);
	step->EnableDistanceControl(firstXDist, isRight);
	steps.push_back(new ConcurrentStep({
		step,
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0),
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
	}, true));


	// Turn and drop elevator
	const double rotateAngleOffset = PrefUtil::getSet("AutoSideSwitchRotateAngle", 45.0) * inv;
	const double turnAngle = startAngle - rotateAngleOffset;
	const double rotateAngleThreshold = PrefUtil::getSet("AutoSideSwitchRotateAngleThreshold", 10.0);
	const int rotateAngleThresholdScans = PrefUtil::getSet("AutoSideSwitchRotateAngleThresholdScans", 5);

	steps.push_back(new ConcurrentStep({
		new Rotate(turnAngle, rotateAngleThreshold, 10.0, rotateAngleThresholdScans),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0.0), true),
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1),
	}, true));


	/************ DOING PICKUP OF SECOND CUBE *****/
	const double xPickupDistance = PrefUtil::getSet("AutoSideSwitchPickupX", -24.0) * inv;
	DoSecondCubePickup(turnAngle, xPickupDistance);

	/************ Score Second Cube *********/
	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kSwitch, DelayParam(DelayParam::DelayType::kNone, 0.0), true));

	const double driveX2 = 0.0;
	const double driveY2 = -0.3;
	steps.push_back(new TimedDrive(turnAngle, driveY2, driveX2, 0.75));

	RunIntakeWithDelay *eject2 = new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.5, -1);
	eject2->SetEjectSpeed(0.65);
	steps.push_back(new ConcurrentStep({
		eject2,
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}));

	/************ Pickup Third Cube **/
	const double driveX3 = 0.0;
	const double driveY3 = 10;
	steps.push_back(new ClosedLoopDrive2(turnAngle, 0.3, driveX3, driveY3, -1, DriveUnit::Units::kInches, 3.0, 0.5, 6));

	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0.0), true));

	DoSecondCubePickup(turnAngle, -24.0 * inv);

	/************ Score Third Cube *********/
	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kLowScale, DelayParam(DelayParam::DelayType::kNone, 0.0), true));

//	const double driveY4 = -0.3;
//	const double driveX4 = 0.5 * inv;
//	steps.push_back(new TimedDrive(turnAngle, driveY4, driveX4, 0.75));
	const double finalAngle = -180.0;
	steps.push_back(new Rotate(finalAngle, rotateAngleThreshold, 10.0, rotateAngleThresholdScans));

	RunIntakeWithDelay *intake4 = new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.5, 1.0);
	intake4->SetIntakeSpeed(-0.50);
	steps.push_back(new ConcurrentStep({
		intake4,
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}));

}


void ChiSideStrategy::DoFirstScale() {
	const double firstDriveSpeed = PrefUtil::getSet("AutoSideScaleSpeed1", 0.5);
	const double firstDriveX = 0.0; // PrefUtil::getSet("AutoSideScaleX1", 15.0) * inv;
	const double firstDriveY = PrefUtil::getSet("AutoSideScaleY1", 313);
	const double distCtrlX = PrefUtil::getSet("AutoSideLongDistCtrlX", 0.25);

	steps.push_back(new PositionElevator(Elevator::ElevatorPosition::kSwitch));

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(
			startAngle, firstDriveSpeed, firstDriveX, firstDriveY, -1,
			DriveUnit::Units::kInches, 5.5, 1.5, 30);
	drive->EnableDistanceControl(distCtrlX, isRight);

	steps.push_back(new ConcurrentStep({
		drive,
		new PositionElevator(Elevator::ElevatorPosition::kHighScale, DelayParam(DelayParam::DelayType::kPosition, 150))
	}, true));
	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, -1.0));
}


void ChiSideStrategy::DoSecondScale() {
	const double secondDriveSpeed = PrefUtil::getSet("AutoSideScaleSpeed2", 0.3);
	const double secondDriveX = PrefUtil::getSet("AutoSideScaleX2", 0.0) * inv;
	const double secondDriveY = PrefUtil::getSet("AutoSideScaleY2", -79.0);
	const double secondDriveRampUp = PrefUtil::getSet("AutoSideScaleRampUp", 0.5);
	const double secondDriveRampDown = PrefUtil::getSet("AutoSideScaleRampDown", 10);
	const double secondDriveElevatorDelay = PrefUtil::getSet("AutoSideScaleElevatorDelay2", 1.0);

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(startAngle, secondDriveSpeed, secondDriveX, secondDriveY, -1, DriveUnit::Units::kInches, 2.75, secondDriveRampUp, secondDriveRampDown);
//	drive->UseDriveDistanceOvershoot(true);

	steps.push_back(new ConcurrentStep({
		drive,
		new PositionMast(Mast::MastPosition::kVertical),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kTime, secondDriveElevatorDelay), false),
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

	/************ DOING PICKUP OF THIRD CUBE *****/
	const double xPickupDistance3 = PrefUtil::getSet("AutoSideScalePickupX.3", -48.0) * inv;
	DoSecondCubePickup(turnAngle, xPickupDistance3);
	DoSecondCubeScale();
}

/**
 * Drives into cube until distance met or auto pickup threshold detected
 */
void ChiSideStrategy::DoSecondCubePickup(double robotAngle, double xDriveDistance) {
	const double speed = PrefUtil::getSet("AutoSidePickupSpeed", 0.2);
	const double driveY = PrefUtil::getSet("AutoSidePickupY", 0.0);
	const double intakeDelayPos = 6;	// delay until closing the intake solenoid to trap the cube

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(robotAngle, speed, xDriveDistance, driveY, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1);
	drive->SetHaltOnIntakePickup(true);
	drive->SetHardStopsContinueFromStep(false);
	steps.push_back(new ConcurrentStep({
		drive,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kPosition, intakeDelayPos), 5.0)
	}, true));
	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1));
}

/**
 * Drives back to known point plus variable offset distance from previous drive step required to pickup
 */
void ChiSideStrategy::DoSecondCubeScale() {
	const double driveAngle = PrefUtil::getSet("AutoSidePickupScaleAngle", -70) * inv;
	const double speed = PrefUtil::getSet("AutoSidePickupScaleSpeed", 0.2);
	const double X = PrefUtil::getSet("AutoSidePickupScaleX", 12) * inv;
	const double Y = PrefUtil::getSet("AutoSidePickupScaleY", 90.0);

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(driveAngle, speed, X, Y, -1, DriveUnit::Units::kInches, 5.0, 0.5, -1);
	drive->UsePickupDistance(isLeft);	// use pickup distance as additional x offset, inverting the direction if we are on the left side
	steps.push_back(new ConcurrentStep({
		drive,
		new PositionElevator(Elevator::ElevatorPosition::kHighScale, DelayParam(DelayParam::DelayType::kTime, 0.5), true)
	}, true));

	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject, DelayParam(DelayParam::DelayType::kNone, 0.0), 2.0, 1.0));
}


void ChiSideStrategy::DoThirdCubePickup(double robotAngle, double xDriveDistance) {
	const double speed = PrefUtil::getSet("AutoSidePickupSpeed", 0.2);
	const double driveY = PrefUtil::getSet("AutoSidePickupY", 0.0);
	const double intakeDelayPos = 2 * fabs(xDriveDistance) / 3;	// delay until closing the intake solenoid to trap the cube

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(robotAngle, speed, xDriveDistance, driveY, -1, DriveUnit::Units::kInches, 4.0, 0.5, -1);
	drive->UsePickupDistance(isRight);	// direction of offset
	drive->SetHaltOnIntakePickup(true);
	drive->SetHardStopsContinueFromStep(false);
	steps.push_back(new ConcurrentStep({
		drive,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kPosition, intakeDelayPos), 5.0)
	}, true));
	steps.push_back(new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Stop, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, -1));
}


void ChiSideStrategy::CrossLine() {
	const double firstDriveSpeed = PrefUtil::getSet("AutoSideTraverseSpeed1", 0.5);
	const double firstDriveY = PrefUtil::getSet("AutoSideTraverseY1", 228.0);
	const double distCtrlX = PrefUtil::getSet("AutoSideLongDistCtrlX", 0.25);

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(
			startAngle, firstDriveSpeed, 0, firstDriveY,
			-1, DriveUnit::Units::kInches, 4.5, 1.5, 30);
	drive->EnableDistanceControl(distCtrlX, isRight);
	steps.push_back(drive);

	const double speed2 = 0.3;
	const double driveX2 = -77 * inv;
	ClosedLoopDrive2 *drive2 = new ClosedLoopDrive2(
			startAngle, speed2, driveX2, 0.0,
			-1, DriveUnit::Units::kInches, 4.5, 1.0, 13);
	steps.push_back(drive2);
}

void ChiSideStrategy::CrossLineHideCubes() {

	const double ejectHeight = 3000;	// this needs measuring for comp-bot

	const double rotate1Angle = -45 * inv;
	Rotate *rotate = new Rotate(rotate1Angle, 5.0, 2.0);
	rotate->SetContinueOnTimeout(true);
	PositionElevator *floor1 = new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0.0));
	floor1->SetOverrideElevatorPosition(ejectHeight);
	steps.push_back(new ConcurrentStep({
		rotate,
		floor1,
	}));
	RunIntakeWithDelay *eject1 = new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject,
					DelayParam(DelayParam::DelayType::kTime, 0.0), 2.0, 1.0);
	eject1->SetEjectSpeed(0.5);
	steps.push_back(eject1);

	///////////////////////////////////////////////////////////////////////////////////
	// Second Cube
	///////////////////////////////////////////////////////////////////////////////////
	const double rotate2Angle = -180 * inv;
	Rotate *rotate2 = new Rotate(rotate2Angle, 5.0, 2.0);
	rotate2->SetContinueOnTimeout(true);
	steps.push_back(rotate2);

	const double pickupSpeed = 0.3;
	ClosedLoopDrive2 *pickupCube2Drive = new ClosedLoopDrive2(rotate2Angle, pickupSpeed, 0.0, -12, -1,
			DriveUnit::Units::kInches, 2.0, -1, -1);
	pickupCube2Drive->SetHaltOnIntakePickup(true);
	steps.push_back(new ConcurrentStep({
		pickupCube2Drive,
		new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Start, DelayParam(DelayParam::DelayType::kNone, 0.0), 0.1, 0.0),
		new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0.0)),
		new IntakeSolenoidWithDelay(true, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0)
	}));


	// Rotate back to score
	const double rotate3Angle = -45 * inv;
	PositionElevator *elev2 = new PositionElevator(Elevator::ElevatorPosition::kFloor, DelayParam(DelayParam::DelayType::kNone, 0.0));
	elev2->SetOverrideElevatorPosition(ejectHeight);
	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(rotate3Angle, pickupSpeed, 0.0, 8, -1, DriveUnit::Units::kInches, 5.0, -1, -1),
		elev2,
	}));
	Rotate *rotate3 = new Rotate(rotate3Angle, 5.0, 3.0);
	rotate3->SetContinueOnTimeout(true);
	steps.push_back(rotate3);

	RunIntakeWithDelay *eject2 = new RunIntakeWithDelay(RunIntakeWithDelay::IntakeState::Eject,
						DelayParam(DelayParam::DelayType::kTime, 0.0), 2.0, 1.0);
	eject2->SetEjectSpeed(0.5);
	steps.push_back(new ConcurrentStep({
		new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 1.0),
		eject2
	}));
}


void ChiSideStrategy::CrossLineNearTraverse() {

	const double delay = PrefUtil::getSet("AutoCrossNearSideDelay", 3.0);
	steps.push_back(new Delay(delay));

	const double speed1 = 0.5;
	const double driveY1 = 24;
	const double driveX1 = -224 * inv;
	const double drive1Timeout = 15;

	ClosedLoopDrive2 *drive = new ClosedLoopDrive2(
			startAngle, speed1, driveX1, driveY1, -1, DriveUnit::Units::kInches,
			drive1Timeout, 0.75, 24);
	steps.push_back(drive);

	const double rotateAngle = -135 * inv;
	Rotate *rotate = new Rotate(rotateAngle, 10.0, 2.0, 0);
	rotate->SetContinueOnTimeout(true);
	steps.push_back(rotate);

	const double angle2 = 90 * inv;
	const double speed2 = 0.4;
	const double driveY2 = 150;
	const double driveX2 = -24 * inv;
	const double drive2Timeout = 15;

	ClosedLoopDrive2 *drive2 = new ClosedLoopDrive2(
			angle2, speed2, driveX2, driveY2, -1, DriveUnit::Units::kInches,
			drive2Timeout, 0.5, 24);
	steps.push_back(drive2);

}
