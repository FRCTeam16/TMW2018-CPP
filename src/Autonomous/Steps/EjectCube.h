/*
 * EjectCube.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_EJECTCUBE_H_
#define SRC_AUTONOMOUS_STEPS_EJECTCUBE_H_

#include <Autonomous/Step.h>
#include <Util/CollisionDetector.h>
#include <RobotMap.h>

class EjectCube: public Step {
public:
	EjectCube(double _delayTime, double _timeout, double _timeToRunEject, double _ejectOnBumpThreshold = -1) :
		timeToDelay(_delayTime),
		timeout(_timeout),
		timeToRunEject(_timeToRunEject) {
		if (_ejectOnBumpThreshold > 0) {
			collisionDetector.reset(new CollisionDetector(RobotMap::gyro, _ejectOnBumpThreshold));
		}
	}
	virtual ~EjectCube() {}
	bool Run(std::shared_ptr<World> world) override;

private:
	const double timeToDelay;
	const double timeout;
	const double timeToRunEject;
	std::unique_ptr<CollisionDetector> collisionDetector;
	double startTime = -1;
	bool ejecting = false;
	bool detectedCollision = false;
	double endEjectTime = -1;
};

#endif /* SRC_AUTONOMOUS_STEPS_EJECTCUBE_H_ */
