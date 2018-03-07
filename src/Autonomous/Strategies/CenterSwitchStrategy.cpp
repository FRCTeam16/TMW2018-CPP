/*
 * CenterSwitchStrategy.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#include <Autonomous/Strategies/CenterSwitchStrategy.h>
#include <Autonomous/World.h>
#include <Autonomous/Steps/ConcurrentStep.h>
#include <Autonomous/Steps/PositionElevator.h>
#include <Autonomous/Steps/PositionMast.h>
#include <Autonomous/Steps/ClosedLoopDrive2.h>
#include <Autonomous/Steps/DriveToBump.h>
#include <Autonomous/Steps/EjectCube.h>
#include <Subsystems/Elevator.h>
#include <Autonomous/Steps/DelayParam.h>

CenterSwitchStrategy::CenterSwitchStrategy(std::shared_ptr<World> world) {
	DoTimedDrive(world.get());
}



void CenterSwitchStrategy::DoTimedDrive(World* world) {
	int inv = 1;
	if (FieldInfo::Location::Left == world->GetFieldInfo().switchLocation) {
		inv = -1;
	}

	// Original was 60" X, 108" Y
	// 0.3 y speed, 0.1665 x
	// invtan = 60.94
	// tan (rads) 1.064

	// atan2(x,y) = 0.5
	//

	const double speed = PrefUtil::getSet("AutoCenterSpeed", 0.35);
	const double yPos = PrefUtil::getSet("AutoCenterY", 108);
	const double xPos = PrefUtil::getSet("AutoCenterX", 60) * inv;
	const double xOffset = PrefUtil::getSet("AutoCenterXOffset", 24);
	const double collisionThreshold = PrefUtil::getSet("AutoCenterCollisionThreshold", 0.5);

	const double xTotal = xPos - xOffset;
	const double angleRadians = atan2(xTotal, yPos);
	const double xSpeed = speed * sin(angleRadians);
	const double ySpeed = speed * cos(angleRadians);

	steps.push_back(
			new ConcurrentStep({
					new DriveToBump(0.0, ySpeed, xSpeed, 4, 1.0, collisionThreshold ),
					new PositionElevator(Elevator::ElevatorPosition::kSwitch),
					new PositionMast(
							Mast::MastPosition::kVertical,
							DelayParam(DelayParam::DelayType::kTime, 0.25), false),
					new EjectCube(1.0,  5.0,  0.5,  collisionThreshold)
	}));
}
