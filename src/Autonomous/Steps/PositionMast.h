/*
 * PositionMast.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_POSITIONMAST_H_
#define SRC_AUTONOMOUS_STEPS_POSITIONMAST_H_

#include <Autonomous/Step.h>
#include <Autonomous/World.h>
#include <Subsystems/Mast.h>
#include "DelayParam.h"

class PositionMast: public Step {
public:
	PositionMast(Mast::MastPosition _pos) :
		position(_pos) {}

	PositionMast(Mast::MastPosition _pos, DelayParam _delay) :
			position(_pos), delay(_delay) {}

	virtual ~PositionMast() {}
	bool Run(std::shared_ptr<World> world) override;

private:
     const Mast::MastPosition position;
     const DelayParam delay;
     bool firstRun = true;
     bool sentPosition = false;
     double target = 0;	// tracks position or time if a delay was requeested
};

#endif /* SRC_AUTONOMOUS_STEPS_POSITIONMAST_H_ */
