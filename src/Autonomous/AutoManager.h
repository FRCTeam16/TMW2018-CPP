/*
 * AutoManager.h
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_AUTOMANAGER_H_
#define SRC_AUTONOMOUS_AUTOMANAGER_H_

#include "WPILib.h"
#include "World.h"
#include "Strategy.h"
#include <map>

class AutoManager {
public:
	AutoManager();
	virtual ~AutoManager();
	void Init(std::shared_ptr<World> world);
	void Periodic(std::shared_ptr<World> world);
	void Instrument();
private:
	enum AutoStrategy {
		kDebug = 0, kCenterSwitch, kScale
	};
	std::shared_ptr<frc::SendableChooser<void*>> strategies;
	std::unique_ptr<Strategy> CreateStrategy(const AutoStrategy &key, std::shared_ptr<World> word);
	std::unique_ptr<Strategy> currentStrategy;
	double startTime = -1;
	bool finalPhaseFired = false;

};

#endif /* SRC_AUTONOMOUS_AUTOMANAGER_H_ */
