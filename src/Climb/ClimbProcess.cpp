/*
 * ClimbProcess.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */
#include <iostream>

#include "ClimbProcess.h"
#include "NoOp.h"
#include "DoPositionElevatorAndMastForClimb.h"
#include "DoClimb.h"
#include "DoLowerRamp.h"
#include "DoCurlUp.h"

ClimbProcess::ClimbProcess() {
}

ClimbProcess::~ClimbProcess() {
}

void ClimbProcess::Run() {
	if (inProgress) {
		activeTransition->Run();
		inProgress = !activeTransition->IsFinished();

		// If we just completed running our transition, set our next state
		if (!inProgress) {
			currentState = nextState;
			activeTransition.reset();
		}
	}
}

void ClimbProcess::Next() {
	if (!inProgress) {
		int nextOrdinal = currentState + 1;
		if (nextOrdinal < NUM_CLIMB_STATES) {
			inProgress = true;
			nextState = static_cast<ClimbState>(nextOrdinal);
			LoadTransition(currentState);
			activeTransition->Initialize(true);
			std::cout << "******************************** next() SET NEXT STATE TO " << nextState << "\n";
		} else {
			// We are at the end transition
		}
	} else {
		std::cout << "Currently in process at state: " << currentState << "\n";
	}
}


void ClimbProcess::Previous() {
	if (!inProgress) {
		int prevOrdinal = currentState - 1;
		if (prevOrdinal >= 0) {
			inProgress = true;
			nextState = static_cast<ClimbState>(prevOrdinal);
			LoadTransition(nextState);	// When reversing, transition based on previous (i.e. next) state action
			activeTransition->Initialize(false);
			std::cout << "******************************** previous() SET NEXT STATE TO " << nextState << "\n";
		}
	} else {
		std::cout << "Currently in process at state: " << currentState << "\n";
	}
}


void ClimbProcess::LoadTransition(ClimbProcess::ClimbState state) {
	StateTransition *transition;
	switch (currentState) {
	case kDisabled:
		transition = new DoPositionElevatorAndMastForClimb();
		break;
	case kInClimbPosition:
		transition = new DoClimb();
		break;
	case kClimbed:
		transition = new DoLowerRamp();
		break;
	case kRotatedDown:
		transition = new DoCurlUp();
		break;
	case kCurledUp:
	default:
		transition = new NoOp();
		break;
	}
	activeTransition.reset(transition);
}

void ClimbProcess::Instrument() {
    SmartDashboard::PutNumber("ClimbProcess State", currentState);
    SmartDashboard::PutBoolean("ClimbProcess Active", inProgress);

}

