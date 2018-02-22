
#ifndef SRC_AUTONOMOUS_STEPS_INTAKESOLENOIDWITHDELAY_H_
#define SRC_AUTONOMOUS_STEPS_INTAKESOLENOIDWITHDELAY_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Autonomous/Steps/DelayParam.h>

class IntakeSolenoidWithDelay: public Step {
public:
	IntakeSolenoidWithDelay(bool _state, DelayParam _delayParam, double _timeout) :
		state(_state),
		delayParam(_delayParam),
		timeout(_timeout) {}
	virtual ~IntakeSolenoidWithDelay() {}
	bool Run(std::shared_ptr<World> world) override;
private:
	const bool state;
	DelayParam delayParam;
	const double timeout;
	bool firstRun = true;
	double target = 0;
	double startedTime = -1;

};

#endif /* SRC_AUTONOMOUS_STEPS_INTAKESOLENOIDWITHDELAY_H_ */
