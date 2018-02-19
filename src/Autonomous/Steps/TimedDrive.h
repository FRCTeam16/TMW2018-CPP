/*
 * TimedDrive.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_TIMEDDRIVE_H_
#define SRC_AUTONOMOUS_STEPS_TIMEDDRIVE_H_

#include <Autonomous/Step.h>

class TimedDrive: public Step {
public:
	TimedDrive(double _angle, double y, double x, double driveTime) :
			angle(_angle),
			ySpeed(y),
			xSpeed(x),
			timeToDrive(driveTime) {}
	virtual ~TimedDrive() {}
	bool Run(std::shared_ptr<World> world) override;

private:
    const double angle;
    const double ySpeed;
    const double xSpeed;
    const double timeToDrive;
    double startTime = -1;
};

#endif /* SRC_AUTONOMOUS_STEPS_TIMEDDRIVE_H_ */
