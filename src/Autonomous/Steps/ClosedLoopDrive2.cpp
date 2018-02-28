#include <Robot.h>
#include <RobotMap.h>
#include <Autonomous/Steps/ClosedLoopDrive2.h>
#include <Autonomous/Steps/RampUtil.h>


ClosedLoopDrive2::~ClosedLoopDrive2() {
}

void ClosedLoopDrive2::setUseCurrentAngle() {
	useCurrentAngle = true;
}

bool ClosedLoopDrive2::Run(std::shared_ptr<World> world) {
	bool doExit = false;
	if (startTime < 0) {
		startTime = world->GetClock();

		if (usePickupDistance) {
			std::cout << "ClosedLoopDrive2 using pickup distance: original = " << XtargetDistance;
			XtargetDistance += DriveUnit::ToInches(world->GetDriveDistance(), DriveUnit::kPulses);
			std::cout << " new = " << XtargetDistance << "\n";
		}

		const double hypotenuse = sqrt(XtargetDistance * XtargetDistance + YtargetDistance * YtargetDistance);
		targetSetpoint = DriveUnit::ToPulses(hypotenuse, units);
		const double targetAngle = (!useCurrentAngle) ? angle : RobotMap::gyro->GetYaw();
		Robot::driveBase->SetTargetAngle(targetAngle);

		std::cout << "Setting Target Drive Distance:" << targetSetpoint << "| Speed:" << speed << "/n";
		Robot::driveBase->SetTargetDriveDistance(targetSetpoint, speed);
		Robot::driveBase->UseClosedLoopDrive();

		startEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();


	}

	const double currentEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	const double currentError = targetSetpoint - currentEncoderPosition;
	const double elapsedTimeSecs = world->GetClock() - startTime;
	const double currentPIDOutput = Robot::driveBase->GetDriveControlOutput();
	double thresholdPassInverter = 1;	// when driving past the threshold, we will invert this value to avoid driving in a negative directoin

	SmartDashboard::PutNumber("PIDController Output", currentPIDOutput);
	std::cout << "XYPIDControlledDrive target setpoint          = " << targetSetpoint << "\n";
	std::cout << "XYPIDControlledDrive Current Encoder Position = " << currentEncoderPosition << "\n";
	std::cout << "XYPIDControlledDrive Current Error            = " << currentError << "\n";
	std::cout << "XYPIDControlledDrive Current Threshold        = " << DriveUnit::ToPulses(distanceThreshold, units) << "\n";
	std::cout << "XYPIDControlledDrive PID Output:              " << currentPIDOutput << "\n";


	std::cout << "Start Time  : " << startTime << "\n";
	std::cout << "Elapsed Time: " << elapsedTimeSecs << "\n";
	std::cout << "Clock: " << world->GetClock() << "\n";


	SmartDashboard::PutNumber("DriveControl P", Robot::driveBase->GetDriveControlP());

	if (distanceThreshold == -1) {
		if (currentEncoderPosition > targetSetpoint) {
			std::cout << "!!!Current encoder passed target\n";
			thresholdPassInverter = -1.0;	// FIXME: Weirdness
			doExit = true;
		}
	} else if (abs(currentError) <= DriveUnit::ToPulses(distanceThreshold, units)) {
		if (thresholdCounter++ >= thresholdCounterTarget) {
			std::cout << "!!!Position reached in " << elapsedTimeSecs << "\n";
			crab->Stop();
			return true;
		}
	} else {
		thresholdCounter = 0;
	}


	StoreDistance(world.get());
	if (elapsedTimeSecs > timeoutCommand) {
		std::cerr << "**** EMERGENCY EXIT OF STEP DUE TO TIMEOUT ***\n";
		crab->Stop();
		return true;
	} else if (elapsedTimeSecs > 1.0 &&  collisionDetector.Detect()) {
		std::cerr << "**** EMERGENCY HALT DUE TO COLLISION ****\n";
		crab->Stop();
		return true;
	} else if (haltOnIntakePickup && Robot::intake->IsPickupTriggered()) {
		std::cout << "!!! Detected Halt on Intake Pickup! \n";
		return true;
	} else {
		const double crabSpeed = speed * ((reverse) ? -1.0 : 1.0);

		RampUtil ru;
		double profiledSpeed = crabSpeed * thresholdPassInverter;
		if (rampUp > 0) {
			profiledSpeed = ru.RampUp(profiledSpeed, elapsedTimeSecs, rampUp);
		}
		if (rampDown > 0) {
			profiledSpeed =  ru.RampDown(profiledSpeed, currentEncoderPosition, targetSetpoint, DriveUnit::ToPulses(rampDown, units));
		}

		const double angleRadians = atan2(XtargetDistance, YtargetDistance);
		const double xspeed = profiledSpeed * sin(angleRadians);
		const double yspeed = profiledSpeed * cos(angleRadians);

		std::cout << "XYPIDController crabSpeed = " << crabSpeed << "\n";
		std::cout << "XYPIDController angleRads = " << angleRadians << "\n";
		std::cout << "XYPIDController xspeed    = " << xspeed << "\n";
		std::cout << "XYPIDController yspeed    = " << yspeed << "\n";


		const double twistOutput =  (elapsedTimeSecs < rampUp) ? 0.0 : Robot::driveBase->GetTwistControlOutput();

		crab->Update(
				(float) twistOutput,
				(float) yspeed,
				(float) xspeed,
				useGyro);
		return doExit;
	}
}

void ClosedLoopDrive2::StoreDistance(World* world) {
	double endDecoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	world->SetDriveDistance(endDecoderPosition - startEncoderPosition);
}


