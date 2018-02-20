/*
 * SideStrategy.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STRATEGIES_SIDESTRATEGY_H_
#define SRC_AUTONOMOUS_STRATEGIES_SIDESTRATEGY_H_

#include <WPILib.h>
#include <Autonomous/Strategy.h>
#include <Autonomous/World.h>

class SideStrategy: public StepStrategy {
public:
	SideStrategy(std::shared_ptr<World> world);
	virtual ~SideStrategy() {}

private:
	bool isLeft = false;
	void DoScaleFirst();
	void DoScaleFirstSecondPickup();
	void DoSwitchFirst();
};

#endif /* SRC_AUTONOMOUS_STRATEGIES_SIDESTRATEGY_H_ */
