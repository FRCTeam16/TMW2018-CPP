/*
 * PositionMast.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#include <Autonomous/Steps/PositionMast.h>
#include <Robot.h>

bool PositionMast::Run(std::shared_ptr<World> world) {
	const bool useDelay = DelayParam::DelayType::kNone != delay.delayType;
	double target = 0;

	if (firstRun) {
		firstRun = false;
		if (!useDelay) {
			Robot::mast->SetMastPosition(position);
			sentPosition = true;
		} else {
			if (DelayParam::DelayType::kTime == delay.delayType) {
				target = world->GetClock() + delay.value;
			} else {
				target = Robot::driveBase->GetDriveControlEncoderPosition() + delay.value;
			}
		}
	}

	bool targetHit = false;
	if (useDelay) {
		switch(delay.delayType) {
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
		Robot::mast->SetMastPosition(position);
		sentPosition = true;
	}

	return sentPosition && Robot::mast->InPosition();

}
