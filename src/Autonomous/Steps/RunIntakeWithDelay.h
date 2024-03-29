#ifndef SRC_AUTONOMOUS_STEPS_RUNINTAKEWITHDELAY_H_
#define SRC_AUTONOMOUS_STEPS_RUNINTAKEWITHDELAY_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Autonomous/Steps/DelayParam.h>

class RunIntakeWithDelay: public Step {
public:
	enum IntakeState { Start, Stop, Eject };

	RunIntakeWithDelay(IntakeState _state, DelayParam _delayParam, double _timeout, double _timeToRun) :
		state(_state), delayParam(_delayParam), timeout(_timeout), timeToRun(_timeToRun) {}
	virtual ~RunIntakeWithDelay() {}
	bool Run(std::shared_ptr<World> world) override;

	void SetEjectSpeed(double _spd) { ejectSpeed = _spd; }
	void SetIntakeSpeed(double _spd) { intakeSpeed = _spd; }

private:
	void ConfigIntake();
	double intakeSpeed = -1.0;
	double ejectSpeed = 1.0;
	const IntakeState state;
	const DelayParam delayParam;
	const double timeout;
	const double timeToRun;			// -1 just turns on the system without exiting
	bool firstRun = true;
	double target = 0;
	double startedTime = -1;
};

#endif /* SRC_AUTONOMOUS_STEPS_RUNINTAKEWITHDELAY_H_ */
