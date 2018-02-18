#include "Robot.h"


std::unique_ptr<OI> Robot::oi;
std::shared_ptr<Ramp> Robot::ramp;
std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<Intake> Robot::intake;
std::shared_ptr<Elevator> Robot::elevator;
std::shared_ptr<Mast> Robot::mast;

void Robot::RobotInit() {
	RobotMap::init();

    ramp.reset(new Ramp());
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

	frc::SmartDashboard::PutData("Auto Modes", &chooser);
}

/**
 * This function is called when the disabled button is hit.
 * You can use it to reset subsystems before shutting down.
 */
void Robot::DisabledInit(){

}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	autonomousCommand = chooser.GetSelected();
	if (autonomousCommand != nullptr)
		autonomousCommand->Start();
}

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
	if (autonomousCommand != nullptr)
		autonomousCommand->Cancel();

	driveBase->InitTeleop();
	climbProcess.release();
	climbProcess.reset(new ClimbProcess());

}

void Robot::TeleopPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	double threshold = 0.1;


	if (oi->GPRB->RisingEdge()) {
		climbProcess->Next();
	} else if (oi->GPLB->RisingEdge()) {
		climbProcess->Previous();
	}

	bool lockWheels = false;
	if (oi->DL4->Pressed()) {
		lockWheels = true;
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


	if (oi->DL3->RisingEdge()) {
		elevator->IncreaseElevatorPosition();
	} else if (oi->DL2->RisingEdge()) {
		elevator->DecreaseElevatorPosition();
	} else if (elevatorDown > threshold) {
		elevator->SetOpenLoopPercent(-elevatorDown);
	} else if (elevatorUp > threshold) {
		elevator->SetOpenLoopPercent(elevatorUp);
	} else {
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

	// 5 - open/close
	// 4 - up/down
	// 6 - shift
	if (oi->DR4->RisingEdge()) {
		intake->ToggleRotateSolenoidState();
	}

	if (oi->DR6->RisingEdge()) {
		elevator->ToggleShifter();
	}


	const bool speedModeTest = oi->DL7->Pressed();
	if (oi->DL7->RisingEdge()) {
		driveBase->UseClosedLoopDrive();
	}
	if (oi->DL7->FallingEdge()) {
		driveBase->UseOpenLoopDrive();
	}

	/*
		Drive Control
	 */
	const double twistInput = oi->GetJoystickTwist(threshold);

	if (speedModeTest) {
		driveBase->SetConstantVelocity(twistInput, 0.38);
	} else if (!lockWheels) {
		driveBase->Crab(
				twistInput,
				-oi->GetJoystickY(threshold),
				oi->GetJoystickX(threshold),
				true);
	} else {
		driveBase->Crab(0, 0, 0, true);
	}

	// ************************************ //


	climbProcess->Run();
	climbProcess->Instrument();
	RunSubsystems();
	InstrumentSubsystems();
}

void Robot::InitSubsystems() {
	Robot::elevator->Init();
	Robot::intake->Init();
	Robot::mast->Init();
}

void Robot::RunSubsystems() {
	Robot::elevator->Run();
	Robot::intake->Run();
	Robot::mast->Run();
}

void Robot::InstrumentSubsystems() {
	Robot::elevator->Instrument();
	Robot::intake->Instrument();
	Robot::mast->Instrument();

}

START_ROBOT_CLASS(Robot);

