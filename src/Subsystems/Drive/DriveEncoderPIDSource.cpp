#include "DriveEncoderPIDSource.h"

DriveEncoderPIDSource::DriveEncoderPIDSource(std::shared_ptr<WPI_TalonSRX> _motor, int *_inverted) {
	motor = _motor;
	inverted = _inverted;
	PIDSource::SetPIDSourceType(PIDSourceType::kDisplacement);
}

DriveEncoderPIDSource::~DriveEncoderPIDSource() {
}

void DriveEncoderPIDSource::SetInitialEncoderValue() {
	initialEncoderValue = motor->GetSelectedSensorPosition(0);
}

double DriveEncoderPIDSource::PIDGet() {
	return fabs(motor->GetSelectedSensorPosition(0) - initialEncoderValue);
}

