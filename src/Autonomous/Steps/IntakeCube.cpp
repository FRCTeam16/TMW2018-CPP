#include <Autonomous/Steps/IntakeCube.h>
#include <Robot.h>


bool IntakeCube::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::intake->Start();
	}

	const double elapsed = currentTime - startTime;

	if (timeUntilClose >= 0 && elapsed > timeUntilClose) {
		Robot::intake->SetExtendSolenoidState(false);
	}

	if (elapsed > timeToRun || Robot::intake->IsPickupTriggered()) {
		std::cout << "IntakeCube:: Stopping step\n";
		Robot::intake->SetExtendSolenoidState(false);
		finished = true;
	}

	if (elapsed > timeout) {
		std::cout << "*** IntakeCube timed out ***\n";
		return true;
	}
	return finished;
}


