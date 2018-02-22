
#include <Autonomous/Steps/EjectCubeWithDelay.h>
#include <Robot.h>
#include <Autonomous/DriveUnit.h>

bool EjectCubeWithDelay::Run(std::shared_ptr<World> world) {
	std::cout << "EjectCubeWithDelay::Run()\n";
	if (firstRun) {
		firstRun = false;
		if (DelayParam::DelayType::kTime == delayParam.delayType) {
			target = world->GetClock() + delayParam.value;
		} else {
			target = Robot::driveBase->GetDriveControlEncoderPosition()
					+ DriveUnit::ToPulses(delayParam.value, DriveUnit::kInches);
		}
		std::cout << "EjectCubeWithDelay Target = " << target << "\n";
	}
	const double currentTime = world->GetClock();

	if (ejectStartedTime < 0) {
		bool targetHit = false;
		switch(delayParam.delayType) {
			case DelayParam::DelayType::kPosition:
				targetHit = Robot::driveBase->GetDriveControlEncoderPosition() >= target;
				break;
			case DelayParam::DelayType::kTime:
				targetHit = currentTime > target;
				break;
			default:
				std::cout << "! EjectCubeWithDelay Warning using default\n";
				targetHit = true;
		}
		if (targetHit) {
			ejectStartedTime = currentTime;
			Robot::intake->Eject();

			// Exit if we're just turning on
			if (timeToRunEject < 0) {
				return true;
			}
		}
	} else {
		if ((currentTime - ejectStartedTime) > timeToRunEject) {
			Robot::intake->Stop();
			return true;
		}
	}
	return false;

}
