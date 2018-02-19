/*
 * IntakeCube.h
 *
 *  Created on: Feb 19, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_INTAKECUBE_H_
#define SRC_AUTONOMOUS_STEPS_INTAKECUBE_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Autonomous/World.h>


class IntakeCube: public Step {
public:
	IntakeCube(double _timeToRun, double _timeUntilClose, double _timeout) :
		timeToRun(_timeToRun),
		timeUntilClose(_timeUntilClose),
		timeout(_timeout)
	{}

	virtual ~IntakeCube() {}
	bool Run(std::shared_ptr<World> world);

private:
	const double timeToRun;
	const double timeUntilClose;	// time until closing solenoid, or -1
	const double timeout;
	double startTime = -1;
	bool finished = false;

};

#endif /* SRC_AUTONOMOUS_STEPS_INTAKECUBE_H_ */
