/*
 * ClimbProcess.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */
#include <Climb/ClimbNoOp.h>
#include <iostream>

#include "ClimbProcess.h"
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
			lastDirection = kNone;
			activeTransition.reset();
			std::cout << frc::Timer::GetFPGATimestamp() << " ClimbProcess | Completed Transition\n";
		}
	}
}

void ClimbProcess::Next() {
	if (lastDirection != kForward) {
		if (inProgress) {
			std::cout << "*** REVERSING ACTIVE CLIMB STEP ***\n";
		}
		int nextOrdinal = (inProgress) ? currentState : currentState + 1;
		if (nextOrdinal < NUM_CLIMB_STATES) {
			std::cout << frc::Timer::GetFPGATimestamp() << " ClimbProcess:Next | Initializing to Run\n";
			inProgress = true;
			nextState = static_cast<ClimbState>(nextOrdinal);
			LoadTransition(currentState);
			activeTransition->Initialize(true);
			lastDirection = kForward;
			std::cout << "******************************** next() SET NEXT STATE TO " << nextState << "\n";
		}
	}
}

void ClimbProcess::DoCurlOverride() {
	inProgress = true;
	nextState = static_cast<ClimbState>(kCurledUp);
	currentState = kRotatedDown;
	LoadTransition(currentState);
	activeTransition->Initialize(true);
	lastDirection = kForward;
	std::cout << "******************************** Manually Set to Do Curl!!" << "\n";
}


void ClimbProcess::Previous() {
	if (lastDirection != kBackward) {
		if (inProgress) {
			std::cout << "*** REVERSING ACTIVE CLIMB STEP ***\n";
		}
		// When reversing, transition based on previous (i.e. next) state action
		int prevOrdinal = (inProgress) ? currentState : currentState - 1;
		if (prevOrdinal >= 0) {
			std::cout << frc::Timer::GetFPGATimestamp() << " ClimbProcess:Previous | Initializing to Run\n";
			inProgress = true;
			nextState = static_cast<ClimbState>(prevOrdinal);
			LoadTransition(nextState);
			activeTransition->Initialize(false);
			lastDirection = kBackward;
			std::cout << "******************************** previous() SET NEXT STATE TO " << nextState << "\n";
		}
	}
}

// FIXME: This needs to use passed in argument, not currentState
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
		transition = new ClimbNoOp();
		break;
	}
	activeTransition.reset(transition);
}


void ClimbProcess::Instrument() {
    SmartDashboard::PutNumber("ClimbProcess State", currentState);
    SmartDashboard::PutBoolean("ClimbProcess Active", inProgress);

}

