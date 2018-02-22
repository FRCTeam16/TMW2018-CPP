
#ifndef SRC_AUTONOMOUS_STEPS_RUNINTAKE_H_
#define SRC_AUTONOMOUS_STEPS_RUNINTAKE_H_

#include <Autonomous/Step.h>
#include <Robot.h>

class RunIntake : public Step {
public:
	RunIntake(bool _start, bool _eject) : start(_start), eject(_eject) {}

	virtual ~RunIntake() {}

	bool Run(std::shared_ptr<World> world) override {
		if (start) {
			Robot::intake->Start();
		} else if (eject) {
			Robot::intake->Eject();
		} else {
			Robot::intake->Stop();
		}
		return true;
	}
private:
	const bool start;
	const bool eject;
};




#endif /* SRC_AUTONOMOUS_STEPS_RUNINTAKE_H_ */
