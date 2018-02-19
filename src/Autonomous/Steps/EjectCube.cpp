#include <Autonomous/Steps/EjectCube.h>
#include <Robot.h>

bool EjectCube::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	bool finished = false;

	// First run
	if (startTime < 0) {
		startTime = currentTime + timeToDelay;
	}
	const double elapsed = currentTime - startTime;
	const bool timedOut = (elapsed > timeout);
	const bool pastMinTime = currentTime >= startTime;


	// If we are using a collision detector, we wait until that hits
	bool doEject = false;
	if (collisionDetector) {
		doEject = (collisionDetector) ?
					(collisionDetector->Detect() && pastMinTime) || timedOut :
					false;
	} else {
		// we just wait for our delay
		doEject = pastMinTime;
	}



	// Wait until to delay to start ejecting or immediately on a bump
	if (!ejecting && doEject) {
		Robot::intake->Eject(1.0);
		ejecting = true;
	}

	// run eject for specified amount of time, then stop
	if (ejecting && elapsed > timeToRunEject) {
		Robot::intake->Stop();
		finished = true;
	}
	return finished;
}


