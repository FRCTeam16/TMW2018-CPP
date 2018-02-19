/*
 * RampUtil.h
 *
 *  Created on: Feb 17, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_RAMPUTIL_H_
#define SRC_AUTONOMOUS_STEPS_RAMPUTIL_H_

class RampUtil {
public:
	RampUtil() {}
    virtual ~RampUtil() {}
    double RampUp(double crabSpeed, double elapsedTime, double ramp);
    double RampDown(double baseSpeed, double currentPosition, double target, double threshold);

};


#endif /* SRC_AUTONOMOUS_STEPS_RAMPUTIL_H_ */

