/*
 * Rotate.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: User
 */

#include <Autonomous/Steps/Rotate.h>
#include <Robot.h>


bool Rotate::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		Robot::driveBase->SetTargetAngle(angle);
		startTime = currentTime;
	}

	const float yaw = RobotMap::gyro->GetYaw();
	const double yawError = Robot::driveBase->GetTwistControlError();
	std::cout << "Rotate(setpoint = " << angle << " current yaw = " << yaw << " error = " << yawError << ")\n";

	if ((currentTime - startTime) > TIMEOUT) {
		std::cerr << "Timed out turning\n";
		crab->Stop();
		return false;
	}

	if (fabs(yawError) <= THRESHOLD ||
		(fabs(yawError) >= (360 - THRESHOLD))) {
		std::cout << "Exiting Turn\n";
		return true;
	} else {
		crab->Update(Robot::driveBase->GetTwistControlOutput(), 0.0, 0.0, true);
		return false;
	}

}
