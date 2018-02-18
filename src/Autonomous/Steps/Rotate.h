/*
 * Rotate.h
 *
 *  Created on: Feb 24, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_STEPS_ROTATE_H_
#define SRC_AUTONOMOUS_STEPS_ROTATE_H_

#include <Autonomous/Step.h>
#include "WPILib.h"

class Rotate : public Step {
private:
	double startTime = -1;
	const double angle = -1;
	const double TIMEOUT = 5.0;
	const double THRESHOLD = 5.0;
public:
	Rotate(double _angle) : angle(_angle) {}
	virtual ~Rotate() {}
	bool Run(std::shared_ptr<World> world);
};

#endif /* SRC_AUTONOMOUS_STEPS_ROTATE_H_ */
