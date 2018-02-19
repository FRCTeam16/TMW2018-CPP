/*
 * Step.h
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_STEP_H_
#define SRC_AUTONOMOUS_STEP_H_

#include "WPILib.h"
#include "World.h"

struct CrabInfo {
	float twist		= 0.0;
	float yspeed	= 0.0;
	float xspeed	= 0.0;
	bool gyro = true;
	bool lock = false;

	void Stop() {
		twist  = 0.0;
		yspeed = 0.0;
		xspeed = 0.0;
	}

	void Update(const float twist_, const float yspeed_, const float xspeed_, const bool gyro_) {
		twist = twist_;
		yspeed = yspeed_;
		xspeed = xspeed_;
		gyro = gyro_;
	}
};

class Step {
public:
	Step();
	virtual ~Step();
	virtual bool Run(std::shared_ptr<World> world) = 0;
	virtual const CrabInfo* GetCrabInfo() { return crab.get(); }
	bool IsManualDriveControl() const { return manualDriveControl; }
protected:
	std::unique_ptr<CrabInfo> crab { new CrabInfo() };
	bool manualDriveControl = false;
};

#endif /* SRC_AUTONOMOUS_STEP_H_ */
