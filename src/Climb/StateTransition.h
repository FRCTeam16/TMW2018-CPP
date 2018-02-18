/*
 * StateTransition.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMB_STATETRANSITION_H_
#define SRC_CLIMB_STATETRANSITION_H_
#include "WPILib.h"

class StateTransition {
public:
	StateTransition() {}
	virtual ~StateTransition() {}

	void Run() {
		if (direction) {
			Forward();
		} else {
			Reverse();
		}
		firstRun = false;
	}

	virtual bool IsFinished() { return true; }

	void Initialize(bool _direction) {
		direction = _direction;
		startTime = frc::Timer::GetFPGATimestamp();
	}

private:
	bool direction = true;	// true is forward, false is reverse
	bool firstRun = true;
	double startTime;

protected:
	virtual void Init() {}
	virtual void Forward() {}
	virtual void Reverse() {}

	bool IsTimeElapsed() {
		return (frc::Timer::GetFPGATimestamp() - startTime) > 0.1;
	}

	bool IsFirstRun() {
		return firstRun;
	}

};




#endif /* SRC_CLIMB_STATETRANSITION_H_ */
