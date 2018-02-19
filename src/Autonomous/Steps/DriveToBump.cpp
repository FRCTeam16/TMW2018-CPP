#include <Autonomous/Steps/DriveToBump.h>
#include <Robot.h>

bool DriveToBump::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->SetTargetAngle(angle);
	}
	if (collisionDetector->Detect() && ((currentTime - startTime) > delayCheckTime)) {
		std::cout << "DriveToBump detected collision\n";
		return true;
	}
	if ((currentTime - startTime) > maxTimeToDrive) {
		std::cerr << "ERROR: DriveToBump timed out\n";
		crab->Stop();
		return true;
	} else {
		crab->Update(
				(float) Robot::driveBase->GetTwistControlOutput(),
				(float) ySpeed,
				(float) xSpeed,
				true);
		return false;
	}
}
