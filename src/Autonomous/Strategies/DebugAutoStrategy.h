/*
 * DebugAutoStrategy.h
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_DEBUGAUTOSTRATEGY_H_
#define SRC_AUTONOMOUS_DEBUGAUTOSTRATEGY_H_

#include "../Strategy.h"

class DebugAutoStrategy : public StepStrategy {
public:
	DebugAutoStrategy();
	virtual ~DebugAutoStrategy() {}

private:
	void DebugShootScoot();
	void JoshDebug();
	void DebugShootScootHang();
};

#endif /* SRC_AUTONOMOUS_DEBUGAUTOSTRATEGY_H_ */
