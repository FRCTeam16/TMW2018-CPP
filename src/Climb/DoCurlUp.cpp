/*
 * DoCurlUp.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#include <Robot.h>
#include <Climb/DoCurlUp.h>

DoCurlUp::DoCurlUp() {
	// TODO Auto-generated constructor stub

}

DoCurlUp::~DoCurlUp() {
	// TODO Auto-generated destructor stub
}

void DoCurlUp::Forward() {
	if (IsFirstRun()) {
		Robot::mast->BeginCurl();
	}

	if (IsTimeElapsed() && Robot::mast->InPosition()) {
		Robot::mast->HoldCurl();
		finished = true;
	}
}

void DoCurlUp::Reverse() {
	if (IsFirstRun()) {
		Robot::mast->UndoHoldCurl();
	}

	if (IsTimeElapsed() && Robot::mast->InPosition()) {
		finished = true;
	}
}

bool DoCurlUp::IsFinished() {
	return finished;
}
