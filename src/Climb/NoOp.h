/*
 * NoOp.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMB_NOOP_H_
#define SRC_CLIMB_NOOP_H_

#include "StateTransition.h"

class NoOp : public StateTransition {
public:
	NoOp();
	virtual ~NoOp();
};

#endif /* SRC_CLIMB_NOOP_H_ */
