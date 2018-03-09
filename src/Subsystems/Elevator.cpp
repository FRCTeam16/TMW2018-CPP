/*
 * Elevator.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */
#include <iostream>
#include <Subsystems/Elevator.h>

Elevator::Elevator() : frc::Subsystem("Elevator") {
	std::cout << "Elevator starting\n";
	for (auto &motor : motors) {
		motor->SetNeutralMode(NeutralMode::Brake);
		motor->ConfigPeakOutputForward(1,  0);
		motor->ConfigPeakOutputReverse(-1, 0);
	}

	mainElevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,0);

	mainElevatorMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 0);
	mainElevatorMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 0);

	followerElevatorMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, mainElevatorMotor->GetDeviceID());
	followerElevatorMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, mainElevatorMotor->GetDeviceID());

	mainElevatorMotor->ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 1, 0, 0, 0);
	followerElevatorMotor->Set(ControlMode::Follower, mainElevatorMotor->GetDeviceID());

	mainElevatorMotor->SetSensorPhase(true);
	magicMotionManager.reset(new ElevatorMagicMotionManager(mainElevatorMotor));
	std::cout << "Elevator() complete\n";
}


Elevator::~Elevator() {
}


void Elevator::Init() {
	ShiftHigh();
	magicMotionManager->Init();
	elevatorPositionThreshold = PrefUtil::getSetInt("ElevatorPositionThreshold", 10);
	SetInitialPosition();
}


void Elevator::SetInitialPosition() {
	runMode = RunMode::kManual;
	openLoopPercent = 0.0;
	mainElevatorMotor->Set(ControlMode::PercentOutput, openLoopPercent);
}


void Elevator::Run() {
	switch (runMode) {
		case kManual:
			mainElevatorMotor->Set(ControlMode::PercentOutput, openLoopPercent);
			break;
		case kMagic:
			magicMotionManager->Run(setpoint);
			break;
	}
	shifter->Set(shifterState ? DoubleSolenoid::Value::kForward : DoubleSolenoid::Value::kReverse);
}


void Elevator::SetOpenLoopPercent(double _openLoopPercent) {
	runMode = RunMode::kManual;
	openLoopPercent = _openLoopPercent;
}

Elevator::ElevatorPosition Elevator::GetElevatorPosition() {
	return elevatorPosition;
}

void Elevator::SetElevatorPosition(ElevatorPosition _elevatorPosition) {
	runMode = RunMode::kMagic;
	elevatorPosition = _elevatorPosition;
	std::cout << "Setting position to: " << elevatorPosition << "\n";
	switch(elevatorPosition) {
		case kFloor:
			setpoint = PrefUtil::getSet("ElevatorPosFloor", 4232);
			break;
		case kSwitch:
			setpoint = PrefUtil::getSet("ElevatorPosSwitch", 18874);
			break;
		case kLowScale:
			setpoint = PrefUtil::getSet("ElevatorPosLowScale", 58690);
			break;
		case kClimb:
			setpoint = PrefUtil::getSet("ElevatorPosClimb", 40332);
			break;
		case kDoClimb:
			setpoint = PrefUtil::getSet("ElevatorPosDoClimb", 18000);
			break;
		case kHighScale:
			setpoint = PrefUtil::getSet("ElevatorPosHighScale", 77226);
			break;
	}
}

void Elevator::SetElevatorSetpoint(int _setpoint) {
	setpoint = _setpoint;
}


bool Elevator::InPosition() {
    double error = setpoint - mainElevatorMotor->GetSelectedSensorPosition(0);
    bool inPosition = (abs(error) < elevatorPositionThreshold);
//	std::cout << "Elevator in position: " << inPosition << "\n";
	return inPosition;
}


void Elevator::IncreaseElevatorPosition() {
	int nextOrdinal = elevatorPosition  + 1;
	if (nextOrdinal < ELEVATOR_POSITION_COUNT) {
		ElevatorPosition nextPosition = static_cast<ElevatorPosition>(nextOrdinal);
		if (ElevatorPosition::kClimb == nextPosition || ElevatorPosition::kDoClimb == nextPosition) {
			nextPosition = ElevatorPosition::kHighScale;
		}
		SetElevatorPosition(nextPosition);
	}
}


void Elevator::DecreaseElevatorPosition() {
	int nextOrdinal = elevatorPosition - 1;
	if (nextOrdinal >= 0 ) {
		ElevatorPosition nextPosition = static_cast<ElevatorPosition>(nextOrdinal);
		if (ElevatorPosition::kClimb == nextPosition || ElevatorPosition::kDoClimb == nextPosition) {
			nextPosition = ElevatorPosition::kLowScale;
		}
		SetElevatorPosition(nextPosition);
	}
}

bool Elevator::IsAboveSwitch() {
	const double threshold = Preferences::GetInstance()->GetDouble("ElevatorPosSwitch", 23900);
	const double current = mainElevatorMotor->GetSelectedSensorPosition(0);
	return current > threshold;
}


void Elevator::HoldPosition() {
	if (RunMode::kManual == runMode) {
		setpoint = mainElevatorMotor->GetSelectedSensorPosition(0);
		runMode = RunMode::kMagic;
	}
}


void Elevator::HoldClimb() {
	for (auto &motor : motors) {
		motor->ConfigPeakOutputForward(0.5, 0);
		motor->ConfigPeakOutputReverse(-0.5, 0);
	};
}


void Elevator::UnholdClimb() {
	for (auto &motor : motors) {
		motor->ConfigPeakOutputForward(1.0, 0);
		motor->ConfigPeakOutputReverse(-1.0, 0);
	};
}


void Elevator::SetShift(bool state) {
	int currentPosition = mainElevatorMotor->GetSelectedSensorPosition(0);
	if (currentPosition < 100) {
		shifterState = state;
	} else {
		std::cerr << "**** IGNORING SHIFT REQUEST - OUT OF SAFETY RANGE ****\n";
	}
}


void Elevator::ToggleShifter() {
	SetShift(!shifterState);
}


void Elevator::ShiftLow() {
	SetShift(false);
}

void Elevator::ShiftHigh() {
	SetShift(true);
}

void Elevator::SetHomePosition() {
	mainElevatorMotor->SetSelectedSensorPosition(0, 0, 0);
	setpoint = 0;
}

void Elevator::Instrument() {
	SmartDashboard::PutNumber("Elevator Position", mainElevatorMotor->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("Elevator Current (Main)", mainElevatorMotor->GetOutputCurrent());
	SmartDashboard::PutNumber("Elevator Current (Follow)", followerElevatorMotor->GetOutputCurrent());
	SmartDashboard::PutNumber("Elevator Output (Main)", mainElevatorMotor->Get());
	SmartDashboard::PutNumber("Elevator Output (Follow)", followerElevatorMotor->Get());
	SmartDashboard::PutBoolean("Elevator Limit Switch FWD", mainElevatorMotor->GetSensorCollection().IsFwdLimitSwitchClosed());
	SmartDashboard::PutBoolean("Elevator Limit Switch REV", mainElevatorMotor->GetSensorCollection().IsRevLimitSwitchClosed());
}
