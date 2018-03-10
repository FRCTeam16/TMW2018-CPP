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
		std::cout << frc::Timer::GetFPGATimestamp() << " DoLowerRamp:Forward | First Run\n";
	}
}

void DoLowerRamp::Reverse() {
	if (IsFirstRun()) {
		Robot::mast->SetMastPosition(Mast::MastPosition::kVertical);
		std::cout << frc::Timer::GetFPGATimestamp() << " DoLowerRamp:Reverse | First Run\n";
	}
}

bool DoLowerRamp::IsFinished() {
	return IsTimeElapsed() && Robot::mast->InPosition();
}

