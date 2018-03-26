#include <iostream>
#include <Autonomous/Steps/RampUtil.h>


double RampUtil::RampUp(double crabSpeed, double elapsedTime, double ramp, double minSpeed) {
	 /*
		 /  |
	   /    |
	  -------
	 */
	double speed = crabSpeed;
	if (elapsedTime < ramp) {
		speed = (crabSpeed / ramp) * elapsedTime;
		std::cout << "RampUp: " << elapsedTime << " Profiled Speed: " << speed << "\n";
		if (speed < minSpeed) {
			speed = minSpeed;
		}
	} else {
		speed = crabSpeed;
	}
	return speed;
}

double RampUtil::RampDown(double baseSpeed, double currentPosition, double target, double threshold, double minSpeed) {
	double speed = baseSpeed;
	double error = target - currentPosition;
	if (abs(error) < abs(threshold) ) {
		speed = baseSpeed * (error / threshold);
		std::cout << "RampDown Error: " << error << " Profiled Speed: " << speed << "\n";
		if (speed < minSpeed) {
			speed = minSpeed;
			std::cout << "RampDown - keeping minimum speed of " << speed;
		}
	}
	return speed;
}
