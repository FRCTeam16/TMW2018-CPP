/*
 * PositionElevator.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_POSITIONELEVATOR_H_
#define SRC_AUTONOMOUS_STEPS_POSITIONELEVATOR_H_

#include <Autonomous/Step.h>
#include <Subsystems/Elevator.h>
#include "DelayParam.h"

class PositionElevator: public Step {
public:
	PositionElevator(Elevator::ElevatorPosition elevatorPosition, DelayParam delayParam) :
		position(elevatorPosition), delayParam(delayParam) {}

	PositionElevator(Elevator::ElevatorPosition elevatorPosition, bool _waitForPosition = false) :
		position(elevatorPosition), waitForPosition(_waitForPosition) {}

	virtual ~PositionElevator() {}
	bool Run(std::shared_ptr<World> world) override;

private:
	const Elevator::ElevatorPosition position;
	DelayParam delayParam;
	bool firstRun = true;
	bool sentPosition = false;
	double target = 0;	// tracks position or time if a delay was requeested
	bool waitForPosition = false;

};

#endif /* SRC_AUTONOMOUS_STEPS_POSITIONELEVATOR_H_ */
