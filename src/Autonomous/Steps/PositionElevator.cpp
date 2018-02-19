
#include <Autonomous/Steps/PositionElevator.h>
#include <Robot.h>
#include "DelayParam.h"

bool PositionElevator::Run(std::shared_ptr<World> world) {
	const bool useDelay = DelayParam::DelayType::kNone != delayParam.delayType;
	std::cout << "Position Elevator using delay? " << useDelay << "\n";

	 if (firstRun) {
		firstRun = false;
		if (!useDelay) {
			std::cout << "Positioning Elevator\n";
			Robot::elevator->SetElevatorPosition(position);
			sentPosition = true;
		} else {
			if (DelayParam::DelayType::kTime == delayParam.delayType) {
				target = world->GetClock() + delayParam.value;
			} else {
				target = Robot::driveBase->GetDriveControlEncoderPosition() + delayParam.value;
			}
		}
	}

	bool targetHit = false;
	if (useDelay) {
		switch(delayParam.delayType) {
			case DelayParam::DelayType::kPosition:
				// FIXME: Only works with positives right now
				targetHit = Robot::driveBase->GetDriveControlEncoderPosition() > target;
				break;
			case DelayParam::DelayType::kTime:
			default:
				targetHit = world->GetClock() > target;
				break;
		}
	}

	if (targetHit) {
		std::cout << "Delay Param target hit, requesting position: " << position << "\n";
		Robot::elevator->SetElevatorPosition(position);
		sentPosition = true;
	}

	return sentPosition && Robot::elevator->InPosition();
}
