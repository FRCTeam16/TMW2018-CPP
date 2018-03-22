#include <Autonomous/Steps/IntakeRotate.h>
#include <Robot.h>

bool IntakeRotate::Run(std::shared_ptr<World> world) {
	const double currentTime = Timer::GetFPGATimestamp();
	if (startTime < 0) {
		startTime = currentTime;
	}

	if ((currentTime - startTime) < timeToRun) {
		Robot::intake->SetRotateIntakeSpeed((direction) ? -1.0 : 1.0);
		return false;
	} else {
		Robot::intake->SetRotateIntakeSpeed(0.0);
		return true;
	}
}
