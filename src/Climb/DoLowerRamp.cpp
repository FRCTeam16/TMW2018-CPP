/*
 * DoLowerRamp.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#include <Robot.h>
#include <Climb/DoLowerRamp.h>

DoLowerRamp::DoLowerRamp() {
}

DoLowerRamp::~DoLowerRamp() {
}

void DoLowerRamp::Forward() {
	if (IsFirstRun()) {
		Robot::mast->SetMastPosition(Mast::MastPosition::kClimb);
	}
}

void DoLowerRamp::Reverse() {
	if (IsFirstRun()) {
		Robot::mast->SetMastPosition(Mast::MastPosition::kVertical);
	}
}

bool DoLowerRamp::IsFinished() {
	return IsTimeElapsed() && Robot::mast->InPosition();
}

