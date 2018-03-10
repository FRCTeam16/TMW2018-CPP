/*
 * DoCurlUp.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#include <Robot.h>
#include <Climb/DoCurlUp.h>

DoCurlUp::DoCurlUp() {
}

DoCurlUp::~DoCurlUp() {
}

void DoCurlUp::Forward() {
	if (IsFirstRun()) {
		Robot::mast->BeginCurl();
		std::cout << frc::Timer::GetFPGATimestamp() << " DoCurlUp:Forward | First Run\n";
	}

	if (IsTimeElapsed() && Robot::mast->InPosition()) {
		Robot::mast->HoldCurl();
		finished = true;
		std::cout << frc::Timer::GetFPGATimestamp() << " DoCurlUp:Forward | COMPLETE\n";
	}
}

void DoCurlUp::Reverse() {
	if (IsFirstRun()) {
		Robot::mast->UndoHoldCurl();
		std::cout << frc::Timer::GetFPGATimestamp() << " DoCurlUp:Reverse | First Run\n";
	}

	if (IsTimeElapsed() && Robot::mast->InPosition()) {
		finished = true;
		std::cout << frc::Timer::GetFPGATimestamp() << " DoCurlUp:Reveres | COMPLETE\n";
	}
}

bool DoCurlUp::IsFinished() {
	return finished;
}
