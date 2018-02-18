/*
 * DoClimb.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMB_DOCLIMB_H_
#define SRC_CLIMB_DOCLIMB_H_

#include <Climb/StateTransition.h>

class DoClimb: public StateTransition {
public:
	DoClimb();
	virtual ~DoClimb();

	void Forward() override;
	void Reverse() override;
	bool IsFinished() override;

private:
	bool finished;
};

#endif /* SRC_CLIMB_DOCLIMB_H_ */
