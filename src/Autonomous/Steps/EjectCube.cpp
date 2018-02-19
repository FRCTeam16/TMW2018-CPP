#include <Autonomous/Steps/EjectCube.h>
#include <Robot.h>

bool EjectCube::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	bool finished = false;

	// First run
	if (startTime < 0) {
		startTime = currentTime;
	}
	const double elapsed = currentTime - startTime;
	const bool timedOut = (elapsed > timeout);
	const bool pastMinTime = elapsed > timeToDelay;


	// If we are using a collision detector, we wait until that hits
	// otherwise wait until min time has passed
	bool doEject = false;

	if (collisionDetector) {
		if (!detectedCollision) {
			detectedCollision = collisionDetector->Detect();
		}
		doEject = (detectedCollision && pastMinTime);
	} else {
		// we just wait for our delay
		doEject = pastMinTime;
	}

	if (timedOut) {
		std::cout << "Timed out so ejecting\n";
		doEject = true;
	}

	std::cout << "EjectCube CollisionDetector => elapsed " << elapsed
			<< " | timedOut? " << timedOut
			<< " | passMinTime? " << pastMinTime
			<< " | doEject? " << doEject << "\n";


	// Wait until to delay to start ejecting or immediately on a bump
	if (!ejecting && doEject) {
		Robot::intake->Eject();	// TODO: FIXME
		ejecting = true;
		endEjectTime = currentTime + timeToRunEject;
	}

	// run eject for specified amount of time, then stop
	if (ejecting && currentTime > endEjectTime) {
		Robot::intake->Stop();
		finished = true;
	}
	return finished;
}


