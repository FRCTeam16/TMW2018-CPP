
#include <Autonomous/Steps/IntakeSolenoidWithDelay.h>
#include <Robot.h>
#include <Autonomous/DriveUnit.h>

bool IntakeSolenoidWithDelay::Run(std::shared_ptr<World> world) {
	std::cout << "IntakeSolenoidWithDelay::Run()\n";
	if (firstRun) {
		firstRun = false;
		if (DelayParam::DelayType::kTime == delayParam.delayType) {
			target = world->GetClock() + delayParam.value;
		} else {
			target = Robot::driveBase->GetDriveControlEncoderPosition()
					+ DriveUnit::ToPulses(delayParam.value, DriveUnit::kInches);
		}
		std::cout << "IntakeSolenoidWithDelay Target = " << target << "\n";
	}
	const double currentTime = world->GetClock();

	bool targetHit = false;
	switch(delayParam.delayType) {
		case DelayParam::DelayType::kPosition:
			targetHit = Robot::driveBase->GetDriveControlEncoderPosition() >= target;
			break;
		case DelayParam::DelayType::kTime:
			targetHit = currentTime > target;
			break;
		default:
			std::cout << "! IntakeSolenoidWithDelay Warning using default\n";
			targetHit = true;
	}
	if (targetHit) {
		std::cout << "IntakeSolenoidWithDelay hit target\n";
		startedTime = currentTime;
		Robot::intake->SetExtendSolenoidState(state);
			return true;
	}
	return false;
}

