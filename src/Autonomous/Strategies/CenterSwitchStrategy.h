/*
 * CenterSwitchStrategy.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STRATEGIES_CENTERSWITCHSTRATEGY_H_
#define SRC_AUTONOMOUS_STRATEGIES_CENTERSWITCHSTRATEGY_H_

#include <Autonomous/Strategy.h>
#include <Autonomous/World.h>

class CenterSwitchStrategy: public StepStrategy {
public:
	CenterSwitchStrategy(std::shared_ptr<World> world);
	virtual ~CenterSwitchStrategy() {}
private:
	void DoTimedDrive(World *world);
};

#endif /* SRC_AUTONOMOUS_STRATEGIES_CENTERSWITCHSTRATEGY_H_ */
