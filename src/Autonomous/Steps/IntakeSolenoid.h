
#ifndef SRC_AUTONOMOUS_STEPS_INTAKESOLENOID_H_
#define SRC_AUTONOMOUS_STEPS_INTAKESOLENOID_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Robot.h>


class IntakeSolenoid : public Step {
public:
	IntakeSolenoid(bool _doOpen, double _delay = 0) : doOpen(_doOpen), delay(_delay) {}
	virtual ~IntakeSolenoid() {}
	bool Run(std::shared_ptr<World> world) {
		std::cout << "IntakeSolenoid::Run(" << doOpen << ")\n";
		const double currentTime = world->GetClock();
		if (firstRun) {
			firstRun = false;
			startTime = currentTime;
		}

		if (currentTime > (startTime + delay)) {
			Robot::intake->SetExtendSolenoidState(doOpen);
			return true;
		} else {
			return false;
		}
	}
private:
	const bool doOpen;
	bool firstRun = true;
	const double delay;
	double startTime = 0;

};

#endif /* SRC_AUTONOMOUS_STEPS_INTAKESOLENOID_H_ */
