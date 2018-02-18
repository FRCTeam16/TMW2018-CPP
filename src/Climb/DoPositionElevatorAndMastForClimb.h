/*
 * DoPositionElevatorAndMastForClimb.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMB_DOPOSITIONELEVATORANDMASTFORCLIMB_H_
#define SRC_CLIMB_DOPOSITIONELEVATORANDMASTFORCLIMB_H_
#include "WPILib.h"
#include "StateTransition.h"

class DoPositionElevatorAndMastForClimb : public StateTransition {
public:
	DoPositionElevatorAndMastForClimb();
	virtual ~DoPositionElevatorAndMastForClimb();

	void Forward() override;
	void Reverse() override;
	bool IsFinished() override;

private:
	bool floorReached = false;
	bool elevateComplete = false;
	bool complete = false;
	long start = 0;

	bool TimeElapsed(double elapsed);
};

#endif /* SRC_CLIMB_DOPOSITIONELEVATORANDMASTFORCLIMB_H_ */
