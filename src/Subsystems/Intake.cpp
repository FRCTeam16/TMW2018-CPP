#include <iostream>
#include <vector>

#include <Robot.h>
#include <Subsystems/Intake.h>
#include <Util/PrefUtil.h>
#include <Subsystems/Elevator.h>

Intake::Intake() : frc::Subsystem("Intake"),
				   rotateCounter(new Counter(RobotMap::intakeRotateEncoder)) {
	std::cout << "Starting Intake\n";
	const std::vector<std::shared_ptr<WPI_VictorSPX>> motors { leftIntakeMotor, rightIntakeMotor };
	for (auto &motor : motors) {
		motor->ConfigPeakOutputForward(1,  0);
		motor->ConfigPeakOutputReverse(-1,  0);
		motor->SetNeutralMode(NeutralMode::Brake);
	}
	std::cout << "Finished Intake\n";
}

Intake::~Intake() {}

void Intake::Init() {
	intakeSpeed = PrefUtil::getSet("IntakeIntakeSpeed", -1.0);
	switchEjectSpeed = PrefUtil::getSet("IntakeEjectSpeedSwitch", 1.0);
	scaleEjectSpeed = PrefUtil::getSet("IntakeEjectSpeedScale", 0.6);
	double intakeAmpThreshold = PrefUtil::getSet("IntakeAmpThreshold", 15);
	int intakeAmpThresholdScans = PrefUtil::getSetInt("IntakeAmpThresholdScans", 5);
	leftIntakeAmpThresholdCounter.reset(new ThresholdCounter(intakeAmpThreshold, intakeAmpThresholdScans));
	rightIntakeAmpThresholdCounter.reset(new ThresholdCounter(intakeAmpThreshold, intakeAmpThresholdScans));

	minRotatePosition = PrefUtil::getSetInt("IntakeMinRotatePos", 0);
	maxRotatePosition = PrefUtil::getSetInt("IntakeMaxRotatePos", 0);

	std::cout << "Intake::Init() Initialized Threshold Counters: "
					  << leftIntakeAmpThresholdCounter.get()
					  << " | "
					  << rightIntakeAmpThresholdCounter.get()
					  << "\n";
}

void Intake::Run() {
	const double currentTime = Timer::GetFPGATimestamp();
	bool performPickup = false;

	if (pickupTriggered && ((currentTime - pickupStartTime) > pickupTimeout)) {
		ResetPickupTriggerState();
	}

	if (IntakeState::kIntake == state && !pickupTriggered) {
		const double leftAmps = GetLeftIntakeCurrent();
		const double rightAmps = GetRightIntakeCurrent();

		if (leftIntakeAmpThresholdCounter->Check(leftAmps) || leftIntakeAmpThresholdCounter->Check(rightAmps)) {
			performPickup = true;
			pickupStartTime = currentTime;
		}
	}

	if (performPickup) {
		std::cout << "*** Intake Pickup Amp Threshold Detected ***\n";
		pickupTriggered = true;
		leftIntakeAmpThresholdCounter->Reset();
		rightIntakeAmpThresholdCounter->Reset();

		/*const bool onFloor = Elevator::ElevatorPosition::kFloor == Robot::elevator->GetElevatorPosition();
		if (onFloor) {
			Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kSwitch);
		}
		SetMotorPercent(0.0);*/
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

	rotateMotor->Set(rotateSpeed);
//	rotateMotor->Set(rotationState.getMotorOutput(rotateCounter));
}

void Intake::Start() {
	state = IntakeState::kIntake;
}

void Intake::Stop() {
	state = IntakeState::kStop;
}

void Intake::Eject() {
	double speed = switchEjectSpeed;
	if (Robot::elevator->IsAboveSwitch()) {
		speed = scaleEjectSpeed;
	}
	Eject(speed);
}

void Intake::Eject(double speed) {
	std::cout << "Intake::Eject " << speed << "\n";
	state = IntakeState::kEject;
	targetEjectSpeed = speed;
	ResetPickupTriggerState();
}

void Intake::SetMotorPercent(double amount) {
	leftIntakeMotor->Set(ControlMode::PercentOutput, amount);
	rightIntakeMotor->Set(ControlMode::PercentOutput, amount);
}

void Intake::SetExtendSolenoidState(bool extend) {
	extendSolenoidState = !extend;
}

void Intake::ToggleExtendSolenoidState() {
	SetExtendSolenoidState(!extendSolenoidState);
}

void Intake::SetRotateIntakeSpeed(double _speed) {
	rotateSpeed = _speed;
}

void Intake::RotateIntakeUp() {
	rotationState.setPosition(maxRotatePosition, true);
}

void Intake::RotateIntakeDown() {
	rotationState.setPosition(minRotatePosition, false);
}

void Intake::SetRotateIntakePosition(int position, bool _direction) {
	rotationState.setPosition(position, _direction);
}



double Intake::GetLeftIntakeCurrent() {
	return RobotMap::powerDistributionPanel->GetCurrent(5);
}

double Intake::GetRightIntakeCurrent() {
	return RobotMap::powerDistributionPanel->GetCurrent(10);
}

void Intake::ResetPickupTriggerState() {
	pickupStartTime = 0.0;
	pickupTriggered = false;
}

void Intake::SetPickupTriggered(bool triggered) {
	pickupTriggered = triggered;
}

bool Intake::IsPickupTriggered() {
	return pickupTriggered;
}

void Intake::Instrument() {
	SmartDashboard::PutNumber("Intake Motor", leftIntakeMotor->Get());
	SmartDashboard::PutBoolean("Intake Solenoid", extendSolenoid->Get());
	SmartDashboard::PutNumber("Rotate Counter", rotateCounter->Get());

	if ((loopCounter++ % 10) == 0) {
		SmartDashboard::PutNumber("Left Intake Current", GetLeftIntakeCurrent());
		SmartDashboard::PutNumber("Right Intake Current", GetRightIntakeCurrent());
	}
}


