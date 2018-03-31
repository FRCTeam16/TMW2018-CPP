#ifndef SRC_AUTONOMOUS_STRATEGIES_CENTERSWITCHSTRATEGY_H_
#define SRC_AUTONOMOUS_STRATEGIES_CENTERSWITCHSTRATEGY_H_

#include "WPILib.h"
#include <Autonomous/Strategy.h>
#include <Autonomous/World.h>

class CenterSwitchStrategy: public StepStrategy {
public:
	CenterSwitchStrategy(std::shared_ptr<World> world);
	virtual ~CenterSwitchStrategy() {}
private:
	void DoTimedDrive(std::shared_ptr<World> world);
};

#endif /* SRC_AUTONOMOUS_STRATEGIES_CENTERSWITCHSTRATEGY_H_ */
