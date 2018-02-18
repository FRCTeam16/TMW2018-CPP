/*
 * Intake.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */
#include <iostream>
#include <vector>

#include <Robot.h>
#include <Subsystems/Intake.h>
#include <Util/PrefUtil.h>
#include <Subsystems/Elevator.h>

Intake::Intake() : frc::Subsystem("Intake") {
	const std::vector<std::shared_ptr<WPI_VictorSPX>> motors { leftIntakeMotor, rightIntakeMotor };
	for (auto &motor : motors) {
		motor->ConfigPeakOutputForward(1,  0);
		motor->ConfigPeakOutputReverse(-1,  0);
		motor->SetNeutralMode(NeutralMode::Brake);
	}
}

Intake::~Intake() {}

void Intake::Init() {
	intakeSpeed = PrefUtil::getSet("IntakeIntakeSpeed", -1.0);
	defaultEjectSpeed = PrefUtil::getSet("IntakeEjectSpeed", 1.0);
	double intakeAmpThreshold = PrefUtil::getSet("IntakeAmpThreshold", 15);
	int intakeAmpThresholdScans = PrefUtil::getSetInt("IntakeAmpThresholdScans", 5);
	leftIntakeAmpThresholdCounter.reset(new ThresholdCounter(intakeAmpThreshold, intakeAmpThresholdScans));
	rightIntakeAmpThresholdCounter.reset(new ThresholdCounter(intakeAmpThreshold, intakeAmpThresholdScans));
}

void Intake::Run() {
	bool performPickup = false;

	if (IntakeState::kIntake == state && !pickupTriggered) {
		const double leftAmps = GetLeftIntakeCurrent();
		const double rightAmps = GetRightIntakeCurrent();
		if (leftIntakeAmpThresholdCounter->Check(leftAmps) || leftIntakeAmpThresholdCounter->Check(rightAmps)) {
			performPickup = true;
		}
	}

	if (performPickup) {
		std::cout << "*** Intake Pickup Amp Threshold Detected ***\n";
		const bool onFloor = Elevator::ElevatorPosition::kFloor == Robot::elevator->GetElevatorPosition();
		if (onFloor) {
			Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kSwitch);
		}
		pickupTriggered = true;
		leftIntakeAmpThresholdCounter->Reset();
		rightIntakeAmpThresholdCounter->Reset();
		SetMotorPercent(0.0);
	} else {
		double targetSpeed = 0.0;
		if (pickupTriggered) {
			targetSpeed = 0;
		} else {
			switch (state) {
				case kIntake:
					targetSpeed = intakeSpeed;
					break;
				case kEject:
					targetSpeed = targetEjectSpeed;
					break;
				case kStop:
					targetSpeed = 0.0;
					break;
			}
		}
		SetMotorPercent(targetSpeed);
	}

	if (extendSolenoid->Get() != extendSolenoidState) {
		extendSolenoid->Set(extendSolenoidState);
	}
	rotateSolenoid->Set(rotateSolenoidState);
}

void Intake::Start() {
	state = IntakeState::kIntake;
}

void Intake::Stop() {
	state = IntakeState::kStop;
}

void Intake::Eject() {
	Eject(defaultEjectSpeed);
}

void Intake::Eject(double speed) {
	state = IntakeState::kEject;
	targetEjectSpeed = speed;
	ResetPickupTriggerState();
}

void Intake::SetMotorPercent(double amount) {
	leftIntakeMotor->Set(ControlMode::PercentOutput, amount);
	rightIntakeMotor->Set(ControlMode::PercentOutput, amount);
}

void Intake::SetExtendSolenoidState(bool extend) {
	extendSolenoidState = extend;
}

void Intake::ToggleExtendSolenoidState() {
	SetExtendSolenoidState(!extendSolenoidState);
}

void Intake::SetRotateSolenoidState(bool rotate) {
	rotateSolenoidState = rotate;
}

void Intake::ToggleRotateSolenoidState() {
	SetRotateSolenoidState(!rotateSolenoidState);
}

double Intake::GetLeftIntakeCurrent() {
	return RobotMap::powerDistributionPanel->GetCurrent(5);
}

double Intake::GetRightIntakeCurrent() {
	return RobotMap::powerDistributionPanel->GetCurrent(10);
}

void Intake::ResetPickupTriggerState() {
	pickupTriggered = false;
}

void Intake::SetPickupTriggered(bool triggered) {
	pickupTriggered = triggered;
}

void Intake::Instrument() {
	SmartDashboard::PutNumber("Intake Motor", leftIntakeMotor->Get());
	SmartDashboard::PutBoolean("Intake Solenoid", extendSolenoid->Get());
	SmartDashboard::PutNumber("Left Intake Current", GetLeftIntakeCurrent());
	SmartDashboard::PutNumber("Right Intake Current", GetRightIntakeCurrent());
}


