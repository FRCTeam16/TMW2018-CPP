#include <Subsystems/Mast.h>
#include <Util/PrefUtil.h>

/** Allocate storage for manager here */
std::shared_ptr<DartMagicMotionManager> DartMagicMotionManager::INSTANCE;

Mast::Mast() : frc::Subsystem("Mast") {
	for (auto &dart : darts) {
		dart->SetSensorPhase(true);
		dart->SetInverted(true);
		dart->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
		dart->SetNeutralMode(NeutralMode::Brake);
		dart->ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
		dart->ConfigPeakOutputForward(1, 0);
		dart->ConfigPeakOutputReverse(-1, 0);
	}

	// Pre-init constraints since they are used for some mapping calculations
	Preferences *prefs = Preferences::GetInstance();
	leftConstraint.reset(new DartConstraint(
			prefs->GetInt("DartLeftMinimum", 159),
			prefs->GetInt("DartLeftMaximum", 875)));
	rightConstraint.reset(new DartConstraint(
			prefs->GetInt("DartRightMinimum", 305),
			prefs->GetInt("DartRightMaximum", 943)));
}

Mast::~Mast() {}

void Mast::Init() {
	Preferences *prefs = Preferences::GetInstance();
	leftConstraint.reset(new DartConstraint(
			prefs->GetInt("DartLeftMinimum", 159),
			prefs->GetInt("DartLeftMaximum", 875)));
	rightConstraint.reset(new DartConstraint(
			prefs->GetInt("DartRightMinimum", 305),
			prefs->GetInt("DartRightMaximum", 943)));

	if (prefs->GetBoolean("DartUseSoftLimit", true)) {
		leftDart->ConfigForwardSoftLimitThreshold(leftConstraint->maximum, 0);
		leftDart->ConfigReverseSoftLimitThreshold(leftConstraint->minimum, 0);
		leftDart->ConfigForwardSoftLimitEnable(true, 0);
		leftDart->ConfigReverseSoftLimitEnable(true, 0);
		rightDart->ConfigForwardSoftLimitThreshold(rightConstraint->maximum, 0);
		rightDart->ConfigReverseSoftLimitThreshold(rightConstraint->minimum, 0);
		rightDart->ConfigForwardSoftLimitEnable(true, 0);
		rightDart->ConfigReverseSoftLimitEnable(true, 0);
	}

	for (auto &dart : darts) {
		dart->ConfigPeakOutputForward(1, 0);
		dart->ConfigPeakOutputReverse(-1, 0);
	}

	dartPositionThreshold = PrefUtil::getSetInt("DartPositionThreshold", 10);
	DART_DIVERGENCE_THRESHOLD = PrefUtil::getSetInt("DartDivergenceThreshold", 50);



	positionLookup[MastPosition::kClimb] = PrefUtil::getSetInt("MastPositionClimb", 175);
	positionLookup[MastPosition::kVertical] = PrefUtil::getSetInt("MastPositionVertical", 450);
	positionLookup[MastPosition::kDrive] = PrefUtil::getSetInt("MastPositionDrive", 800);
	positionLookup[MastPosition::kCurl] = PrefUtil::getSetInt("MastPositoinCurl", 550);
	positionLookup[MastPosition::kStartingPosition] = PrefUtil::getSetInt("MastPosition", 862);


	runMode = RunMode::kMagic;
	magicTarget = leftDart->GetSelectedSensorPosition(0);

	if (magicMotionManager == nullptr) {
		magicMotionManager.reset(DartMagicMotionManager::GetInstance(leftDart, leftConstraint, rightDart, rightConstraint).get());
	}

	magicMotionManager->InitDefaultPID();
	leftDart->SelectProfileSlot(0, 0);
}


void Mast::Run() {
	bool dartsDiverged = AreDartsDiverged();

	switch (runMode) {
		case kManual:
			magicMotionManager->Stop();
			leftDart->Set(ControlMode::PercentOutput, leftMotorSpeed);
			rightDart->Set(ControlMode::PercentOutput, rightMotorSpeed);
			break;
		case kPositionControl:
			leftDart->Set(ControlMode::Position, magicTarget);
			rightDart->Set(ControlMode::Position, DartConstraint::mapDartConstraint(magicTarget, leftConstraint, rightConstraint));
			magicMotionManager->Run(magicTarget);
			break;
		case kMagic:
			if (dartsDiverged) {
				std::cerr << "MAST DARTS DISABLED DUE TO POSITION DIVERGENCE\n";
				runMode = RunMode::kManual;
				leftDart->Set(ControlMode::PercentOutput, 0);
				rightDart->Set(ControlMode::PercentOutput, 0);
				return;
			}
			magicMotionManager->Run(magicTarget);
			break;
	}

}


void Mast::SetMastPosition(Mast::MastPosition _mastPosition) {
	mastPosition = _mastPosition;
	int target;
	// FIXME: Switch default to main case, others to default
	switch (mastPosition) {
		case kClimb:
		case kVertical:
		case kDrive:
		case kCurl:
		case kStartingPosition:
			target = positionLookup[mastPosition];
			break;
		case kManualPosition:
		default:
			// Just use current magic target
			target = magicTarget;
			break;
	}
	runMode = Mast::RunMode::kMagic;
	std::cout << "Setting mast to " << mastPosition << " pos: " << target << "\n";
	magicTarget = target;
}

bool Mast::InPosition() {
	int leftError = magicTarget - leftDart->GetSelectedSensorPosition(0);
	bool inPosition = (abs(leftError) < dartPositionThreshold);

	int rightError = magicTarget - DartConstraint::mapDartConstraint(rightDart->GetSelectedSensorPosition(0),
			rightConstraint, leftConstraint);
	bool rightInPosition = (abs(rightError) < dartPositionThreshold);

	std::cout << "Darts in Position? L " << inPosition << " | R " << rightInPosition << "\n";
	return inPosition && rightInPosition;
}


void Mast::BeginCurl() {
	std::cout << "************ BEGINNING CURL **************\n";
	magicMotionManager->InitCurlPID();
	SetMastPosition(Mast::MastPosition::kCurl);
}


void Mast::HoldCurl() {
	std::cout << "************ HOLDING CURL **************\n";
	SetLeftMotorSpeed(0.0);
	SetRightMotorSpeed(0.0);
}


void Mast::UndoHoldCurl() {
	std::cout << "************ Undoing CURL **************\n";
	magicMotionManager->InitDefaultPID();
	leftDart->SelectProfileSlot(0, 0);
	rightDart->SelectProfileSlot(0, 0);
	SetMastPosition(Mast::MastPosition::kClimb);
}


bool Mast::AreDartsDiverged() {
	int leftPosition = leftDart->GetSelectedSensorPosition(0);
	int rightPosition = rightDart->GetSelectedSensorPosition(0);

	int rightTarget = DartConstraint::mapDartConstraint(leftPosition, leftConstraint, rightConstraint);

	int mappedDifference = abs(rightPosition - rightTarget);
	SmartDashboard::PutNumber("Dart Divergence (RD)", mappedDifference);
	if (mappedDifference > DART_DIVERGENCE_THRESHOLD) {
		return true;
	} else {
		return false;
	}
}


void Mast::SetTarget(int target) {
	SetMastPosition(Mast::MastPosition::kManualPosition);
	magicTarget = target;
}

void Mast::AdjustTarget(int ticks) {
	runMode = Mast::RunMode::kPositionControl;
	SetTarget(magicTarget + ticks);
}


void Mast::SetLeftMotorSpeed(double motorSpeed) {
	runMode = Mast::RunMode::kManual;
	leftMotorSpeed = motorSpeed;
}

void Mast::SetRightMotorSpeed(double motorSpeed) {
	runMode = Mast::RunMode::kManual;
	rightMotorSpeed = motorSpeed;
}

void Mast::ZeroLeftMotorSpeedIfManual() {
	if (Mast::RunMode::kManual == runMode) {
		leftMotorSpeed = 0.0;
	}
}


void Mast::Instrument() {
	SmartDashboard::PutNumber("Left Dart", leftDart->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("Right Dart", rightDart->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("Mast Position", mastPosition);	// TODO: show name
	SmartDashboard::PutNumber("Left Dart Amps", leftDart->GetOutputCurrent());
	SmartDashboard::PutNumber("Right Dart Amps", rightDart->GetOutputCurrent());
	AreDartsDiverged();

	SmartDashboard::PutNumber("Left Dart V", leftDart->ConfigGetParameter(ParamEnum::eMotMag_VelCruise, 0, 0));
	SmartDashboard::PutNumber("Left Dart A", leftDart->ConfigGetParameter(ParamEnum::eMotMag_Accel, 0, 0));
	SmartDashboard::PutNumber("Left Dart P", leftDart->ConfigGetParameter(ParamEnum::eProfileParamSlot_P, 0, 0));
}
