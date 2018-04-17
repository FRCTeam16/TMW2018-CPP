#include "Robot.h"


std::unique_ptr<OI> Robot::oi;
std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<Intake> Robot::intake;
std::shared_ptr<Elevator> Robot::elevator;
std::shared_ptr<Mast> Robot::mast;

void Robot::RobotInit() {
	RobotMap::init();

	driveBase.reset(new DriveBase());
	intake.reset(new Intake());
	elevator.reset(new Elevator());
	mast.reset(new Mast());


	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi.reset(new OI());

	autoManager.reset(new AutoManager());
	collisionDetector.reset(new CollisionDetector(RobotMap::gyro, 10));

	telemetryLogger.reset(new TelemetryLogger());
//	telemetryLogger->Launch();
//
	statusReporter.reset(new StatusReporter());
	statusReporter->Launch();
	dmsProcessManager.reset(new DmsProcessManager(statusReporter));


	std::cout << "Robot::RobotInit() complete - stratofortress is aloft\n";
}

/**
 * This function is called when the disabled button is hit.
 * You can use it to reset subsystems before shutting down.
 */
void Robot::DisabledInit(){
	telemetryLogger->End();
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	InstrumentSubsystems();
}

void Robot::AutonomousInit() {
/*	autonomousCommand = chooser.GetSelected();
	if (autonomousCommand != nullptr)
		autonomousCommand->Start();
*/
	InitSubsystems();
	telemetryLogger->Begin();

	intake->SetExtendSolenoidState(true);
	world.reset(new World());

	//
	// BEGIN PRACTICE DATA AUTO OVERRIDE
	//
	if (!Preferences::GetInstance()->ContainsKey("AutoOverride")) {
		Preferences::GetInstance()->PutString("AutoOverride", "");
	}
	FieldInfo overrideField = FieldInfo(Preferences::GetInstance()->GetString("AutoOverride", ""));
	if (FieldInfo::Location::Unknown != overrideField.switchLocation &&
		FieldInfo::Location::Unknown != overrideField.scaleLocation &&
		FieldInfo::Location::Unknown != overrideField.farSwitchLocation) {
		world->SetFieldInfo(overrideField);
	}

	//
	// END PRACTICE DATA AUTO OVERRIDE
	//

	autoManager->Init(world);
}

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	autoManager->Periodic(world);
	RunSubsystems();
	InstrumentSubsystems();
}

void Robot::TeleopInit() {
	std::cout << "Robot::TeleopInit\n";
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
	if (autonomousCommand != nullptr)
		autonomousCommand->Cancel();

	telemetryLogger->End();
	driveBase->InitTeleop();
	climbProcess.release();
	climbProcess.reset(new ClimbProcess());
	InitSubsystems();
	std::cout << "Completed TeleopInit\n";
}

void Robot::TeleopPeriodic() {
	double startTime = frc::Timer::GetFPGATimestamp();
	frc::Scheduler::GetInstance()->Run();
	double threshold = 0.1;


	if (oi->GPRB->RisingEdge()) {
		climbProcess->Next();
		std::cout << frc::Timer::GetFPGATimestamp() << " GPRB Rising Edge\n";
	} else if (oi->GPLB->RisingEdge()) {
		climbProcess->Previous();
		std::cout << frc::Timer::GetFPGATimestamp() << " GPLB Rising Edge\n";
	} else if (OI::DPad::kUp == oi->GetGamepadDPad()) {
		climbProcess->DoCurlOverride();
		std::cout << frc::Timer::GetFPGATimestamp() << " DPAD-D\n";
	}

	bool lockWheels = false;
	if (oi->DL6->Pressed()) {
		lockWheels = true;
	}
	if (oi->DR9->Pressed()){
		intake->SetExtendSolenoidState2(false);

	}

	/*
	 * Mast Dart Control
	 */
	double leftInput = oi->GetGamepadLeftStick();

	if (oi->GPX->RisingEdge()) {
		mast->SetMastPosition(Mast::MastPosition::kDrive);
	} else if (oi->GPA->RisingEdge()) {
		mast->SetMastPosition(Mast::MastPosition::kVertical);
	} else if (oi->GPY->RisingEdge()) {
		mast->SetMastPosition(Mast::MastPosition::kClimb);
	} else if (oi->GPB->RisingEdge()) {
		mast->SetMastPosition(Mast::MastPosition::kStartingPosition);
	} else if (abs(leftInput) > threshold) {
		// 80 ticks / second
		// 50 scans / escond
		// 100 scans max velocity
		int ticks = (int) (leftInput * 2);
		mast->AdjustTarget(ticks);
	} else {
		mast->ZeroLeftMotorSpeedIfManual();
	}



	/*
	 * Elevator Control
	 */

	const double elevatorDown = oi->GetGamepadLT();
	const double elevatorUp = oi->GetGamepadRT();


	if (OI::DPad::kDown == oi->GetGamepadDPad()) {
		elevator->SetHomePosition();
	}


	if (oi->DL3->RisingEdge()) {
		elevator->SetElevatorPosition(Elevator::kflipCube);
	} else if (oi->DL2->RisingEdge()) {
		elevator->SetElevatorPosition(Elevator::kFloor);
	} else if (elevatorDown > threshold) {
		elevator->SetOpenLoopPercent(-elevatorDown);
	} else if (elevatorUp > threshold) {
		elevator->SetOpenLoopPercent(elevatorUp);
	}
	/*else if (OI::DPad::kUp == oi->GetGamepadDPad()) {
		elevator->SetElevatorSetpoint(PrefUtil::getSet("ElevatorPosPortal", 19203));
	}*/
	else {
		elevator->HoldPosition();
	}


	/*
	 * Intake Control
	 */
	if (oi->DR1->RisingEdge()) {
		intake->ResetPickupTriggerState();
	}
	if (oi->DR1->Pressed()){
		intake->Start();
	} else if (oi->DR2->Pressed()) {
		intake->Eject();
	} else {
		intake->Stop();

			}


	if (oi->DL1->RisingEdge()) {
		intake->SetExtendSolenoidState(true);
	} else if (oi->DL1->FallingEdge()) {
		intake->SetExtendSolenoidState(false);
	}

	double intakeRotateSpeed = 0.0;
	if (OI::DPad::kLeft == oi->GetGamepadDPad()) {
		intakeRotateSpeed = 1.0;
	}
	if (OI::DPad::kRight == oi->GetGamepadDPad()) {
		intakeRotateSpeed = -1.0;
	}
	intake->SetRotateIntakeSpeed(intakeRotateSpeed);


	// 5 - open/close
	// 6 - shift


	if (oi->DR6->RisingEdge()) {
		elevator->ToggleShifter();
	}


	/**
	 * Testing and Diagnostics
	 */
	const bool speedModeTest = oi->DL7->Pressed();
	const bool dmsMode = oi->DL11->Pressed();
	dmsProcessManager->SetRunning(dmsMode);

	/*
		Drive Control
	 */
	 double twistInput = oi->GetJoystickTwist(threshold);
	if (oi->DL4->Pressed()) {
		driveBase->SetTargetAngle(-45.0);
		twistInput = driveBase->GetCrabTwistOutput();
	} else 	if (oi->DL5->Pressed()) {
		driveBase->SetTargetAngle(45.0);
		twistInput = driveBase->GetCrabTwistOutput();
	}

	double start = frc::Timer::GetFPGATimestamp();
	if (speedModeTest) {
		driveBase->SetConstantVelocity(twistInput, 0.60);
		driveBase->Diagnostics();
	} else if (dmsMode) {
		// DriveBase input handled elsewhere
	} else if (!lockWheels) {
		driveBase->Crab(
				twistInput,
				-oi->GetJoystickY(threshold),
				oi->GetJoystickX(threshold),
				true);
	} else {
		driveBase->Crab(0, 0, 0, true);
	}

	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("DriveBaseRun", (now-start) * 1000);


	// ************************************ //

	climbProcess->Run();
	climbProcess->Instrument();

	dmsProcessManager->Run();

	RunSubsystems();
	InstrumentSubsystems();

	long elapsed = (frc::Timer::GetFPGATimestamp() - startTime) * 1000.0;
	SmartDashboard::PutNumber("Teleop Period (ms)", elapsed);
}

void Robot::InitSubsystems() {
	Robot::elevator->Init();
	Robot::intake->Init();
	std::cout << "Mast init start\n";
	Robot::mast->Init();
	std::cout << "Mast init done\n";
}

void Robot::RunSubsystems() {
	double start = frc::Timer::GetFPGATimestamp();
	Robot::elevator->Run();
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("ElevatorRun", (now-start) * 1000);

	start = frc::Timer::GetFPGATimestamp();
	Robot::intake->Run();
	now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("IntakeRun", (now-start) * 1000);

	start = frc::Timer::GetFPGATimestamp();
	Robot::mast->Run();
	now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("MastRun", (now-start) * 1000);

}

void Robot::InstrumentSubsystems() {
	Robot::elevator->Instrument();
	Robot::intake->Instrument();
	Robot::mast->Instrument();
	Robot::driveBase->Instrument();

	autoManager->Instrument();

	collisionDetector->Detect();

	double now = frc::Timer::GetFPGATimestamp();
	double elapsedMs = (now - lastTime) * 1000;
	SmartDashboard::PutNumber("Period (ms)", elapsedMs);
	lastTime = now;
}

START_ROBOT_CLASS(Robot);

