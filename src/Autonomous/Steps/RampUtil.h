/*
 * RampUtil.h
 *
 *  Created on: Feb 17, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_STEPS_RAMPUTIL_H_
#define SRC_AUTONOMOUS_STEPS_RAMPUTIL_H_

class RampUtil {
private:
    const int threshold;

public:
    RampUtil(int positionThreshold) : threshold(positionThreshold) {}
    virtual ~RampUtil() {}
    double RampUp(double crabSpeed, double elapsedTime);
    double RampDown(double baseSpeed, double currentPosition, double target);
};

#endif /* SRC_AUTONOMOUS_STEPS_RAMPUTIL_H_ */

