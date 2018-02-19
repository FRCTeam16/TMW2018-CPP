
#ifndef SRC_AUTONOMOUS_STEPS_INTAKESOLENOID_H_
#define SRC_AUTONOMOUS_STEPS_INTAKESOLENOID_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Robot.h>


class IntakeSolenoid : public Step {
public:
	IntakeSolenoid(bool _doOpen) : doOpen(_doOpen) {}
	virtual ~IntakeSolenoid() {}
	bool Run(std::shared_ptr<World> world) {
		if (firstRun) {
			Robot::intake->SetExtendSolenoidState(doOpen);
			firstRun = false;
		}
		return true;
	}
private:
	const bool doOpen;
	bool firstRun = true;

};

#endif /* SRC_AUTONOMOUS_STEPS_INTAKESOLENOID_H_ */
