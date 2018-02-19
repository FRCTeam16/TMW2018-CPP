#include <Autonomous/Steps/DriveToBump.h>
#include <Robot.h>

bool DriveToBump::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->SetTargetAngle(angle);
	}
	const double elapsed = (currentTime - startTime);

	if (!collisionDetected) {
		collisionDetected = collisionDetector->Detect();
	}
	if (collisionDetected && (elapsed > delayCheckTime)) {
		std::cout << "DriveToBump detected collision\n";
		crab->Stop();
		return true;
	}

	if (elapsed > maxTimeToDrive) {
		std::cerr << "ERROR: DriveToBump timed out\n";
		crab->Stop();
		return true;
	} else {
		std::cout << "DriveToBump -> updating driveBase\n";
		crab->Update(
				(float) Robot::driveBase->GetTwistControlOutput(),
				(float) ySpeed,
				(float) xSpeed,
				true);
		return false;
	}
}
