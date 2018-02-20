/*
 * ClimbProcess.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMBPROCESS_H_
#define SRC_CLIMBPROCESS_H_
#include "WPILib.h"
#include "StateTransition.h"


class ClimbProcess {
public:
	ClimbProcess();
	virtual ~ClimbProcess();

	enum ClimbState {
		kDisabled, kInClimbPosition, kClimbed, kRotatedDown, kCurledUp
	};
	const int NUM_CLIMB_STATES = 5;

	void Run();
	void Instrument();
	void Next();
	void Previous();

private:
	ClimbState currentState = ClimbState::kDisabled;
	ClimbState nextState = ClimbState::kDisabled;
	std::shared_ptr<StateTransition> activeTransition;
	bool inProgress = false;
	void LoadTransition(ClimbProcess::ClimbState climbState);

	enum Direction {
		kForward, kBackward, kNone
	};
	Direction lastDirection = kNone;
};

#endif /* SRC_CLIMBPROCESS_H_ */
