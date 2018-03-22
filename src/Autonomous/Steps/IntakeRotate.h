#ifndef SRC_AUTONOMOUS_STEPS_INTAKEROTATE_H_
#define SRC_AUTONOMOUS_STEPS_INTAKEROTATE_H_

#include "WPILib.h"
#include <Autonomous/Step.h>

class IntakeRotate: public Step {
public:
	IntakeRotate(bool _dir, double _timeToRun) :
		direction(_dir), timeToRun(_timeToRun) {}
	virtual ~IntakeRotate() {}
	bool Run(std::shared_ptr<World> world);

private:
	const bool direction;
	const double timeToRun;
	double startTime = -1;
};

#endif /* SRC_AUTONOMOUS_STEPS_INTAKEROTATE_H_ */
