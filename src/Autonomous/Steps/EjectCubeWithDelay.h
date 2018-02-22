/*
 * EjectCubeWithDelay.h
 *
 *  Created on: Feb 21, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_EJECTCUBEWITHDELAY_H_
#define SRC_AUTONOMOUS_STEPS_EJECTCUBEWITHDELAY_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Autonomous/Steps/DelayParam.h>

class EjectCubeWithDelay: public Step {
public:
	EjectCubeWithDelay(DelayParam _delayParam, double _timeout, double _timeToRunEject) :
			delayParam(_delayParam),
			timeout(_timeout),
			timeToRunEject(_timeToRunEject) {}
	virtual ~EjectCubeWithDelay() {}
	bool Run(std::shared_ptr<World> world) override;

private:
	DelayParam delayParam;
	const double timeout;
	const double timeToRunEject;
	bool firstRun = true;
	double target = 0;
	double ejectStartedTime = -1;

};

#endif /* SRC_AUTONOMOUS_STEPS_EJECTCUBEWITHDELAY_H_ */
