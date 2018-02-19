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
	if (startTime < 0) {
		startTime = world->GetClock();
		const double hypotenuse = sqrt(XtargetDistance * XtargetDistance + YtargetDistance * YtargetDistance);
		targetSetpoint = DriveUnit::ToPulses(hypotenuse, units);
		const double targetAngle = (!useCurrentAngle) ? angle : RobotMap::gyro->GetYaw();
		Robot::driveBase->SetTargetAngle(targetAngle);

		Robot::driveBase->SetTargetDriveDistance(targetSetpoint, speed);
		Robot::driveBase->UseClosedLoopDrive();
	}

	const double currentEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	const double currentError = Robot::driveBase->GetDriveControlError();
	const double elapsedTimeSecs = world->GetClock() - startTime;
	const double currentPIDOutput = Robot::driveBase->GetDriveControlOutput();

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
			std::cout << "Current encoder passed target\n";
			return true;
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

	if (elapsedTimeSecs > timeoutCommand) {
		std::cerr << "**** EMERGENCY EXIT OF STEP DUE TO TIMEOUT ***\n";
		crab->Stop();
		return true;
	} else if (elapsedTimeSecs > 1.0 &&  collisionDetector.Detect()) {
		std::cerr << "**** EMERGENCY HALT DUE TO COLLISION ****\n";
		crab->Stop();
		return true;
	} else {
		const double crabSpeed = currentPIDOutput * ((reverse) ? -1.0 : 1.0);

		RampUtil ru;
		double profiledSpeed = crabSpeed;
		if (rampUp > 0) {
			profiledSpeed = ru.RampUp(crabSpeed, elapsedTimeSecs, rampUp);
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


		crab->Update(
				(float) Robot::driveBase->GetTwistControlOutput(),
				(float) yspeed,
				(float) xspeed,
				useGyro);
		return false;
	}
}


