/*
 * DoLowerRamp.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMB_DOLOWERRAMP_H_
#define SRC_CLIMB_DOLOWERRAMP_H_

#include <Climb/StateTransition.h>

class DoLowerRamp: public StateTransition {
public:
	DoLowerRamp();
	virtual ~DoLowerRamp();

	void Forward() override;
	void Reverse() override;
	bool IsFinished() override;
};

#endif /* SRC_CLIMB_DOLOWERRAMP_H_ */
