#include <iostream>
#include <Autonomous/Steps/RampUtil.h>


double RampUtil::RampUp(double crabSpeed, double elapsedTime) {
	 /*
		 /  |
	   /    |
	  -------
	 */
	const double ramp = 0.5;
	double speed = crabSpeed;
	if (elapsedTime < ramp) {
		speed = (crabSpeed / ramp) * elapsedTime;
		std::cout << "RampUp: " << elapsedTime << " Profiled Speed: " << speed << "\n";
		if (speed < 0.10) {
			speed = 0.10;
		}
	} else {
		speed = crabSpeed;
	}
	return speed;
}

double RampUtil::RampDown(double baseSpeed, double currentPosition, double target) {
	double speed = baseSpeed;
	double error = target - currentPosition;
	if (abs(error) < abs(threshold) ) {
		speed = baseSpeed * (error / threshold);
		std::cout << "RampDown Error: " << error << " Profiled Speed: " << speed << "\n";
		if (speed < 0.10) {
			speed = 0.10;
			std::cout << "RampDown - keeping minimum speed of " << speed;
		}
	}
	return speed;
}
