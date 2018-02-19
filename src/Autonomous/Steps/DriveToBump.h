/*
 * DriveToBump.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_DRIVETOBUMP_H_
#define SRC_AUTONOMOUS_STEPS_DRIVETOBUMP_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Autonomous/World.h>
#include <Util/CollisionDetector.h>
#include <RobotMap.h>


class DriveToBump : public Step {
public:
	DriveToBump(double _angle, double _ySpeed, double _xSpeed, double _maxDriveTime,
			    double _delayCheckTime, double _gThreshold) :
		angle(_angle),
		ySpeed(_ySpeed),
		xSpeed(_xSpeed),
		maxTimeToDrive(_maxDriveTime),
		delayCheckTime(_delayCheckTime),
		collisionDetector(new CollisionDetector(RobotMap::gyro, _gThreshold)) {}

	virtual ~DriveToBump() {}
	bool Run(std::shared_ptr<World> world) override;

private:
     const double angle;
     const double ySpeed;
     const double xSpeed;
     const double maxTimeToDrive;
     const double delayCheckTime;	// delay before checking for collision
     const std::unique_ptr<CollisionDetector> collisionDetector;

     double startTime = -1;
     bool collisionDetected = false;
};

#endif /* SRC_AUTONOMOUS_STEPS_DRIVETOBUMP_H_ */
