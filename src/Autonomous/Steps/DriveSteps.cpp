/*
 * PIDDrive.cpp
 */

#include <Autonomous/Steps/DriveSteps.h>
#include <Robot.h>

using namespace std;


bool TimedDrive::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->UseOpenLoopDrive();
		Robot::driveBase->SetTargetAngle(angle);
	}
	if ((currentTime - startTime) > timeToDrive) {
		Robot::driveBase->UseClosedLoopDrive();
		return true;
	} else {
		crab->Update(Robot::driveBase->GetTwistControlOutput(), ySpeed, xSpeed, true);
		return false;
	}
}

bool DriveToBump::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->SetTargetAngle(angle);
	}
	if (collisionDetector->Detect() && ((currentTime - startTime) > ignoreTime)) {
		std::cout << "DriveToBump detected collision\n";
		return true;
	}
	if ((currentTime - startTime) > maxTimeToDrive) {
		std::cerr << "DriveToBump timed out\n";
		crab->Stop();
		return true;
	} else {
		crab->Update(Robot::driveBase->GetTwistControlOutput(), ySpeed, xSpeed, true);
		return false;
	}
}



bool SimpleEncoderDrive::Run(std::shared_ptr<World> world) {
	cout << "SimpleEncoderDrive target distance" << targetDistance << '\n';

	// Initialize
	if (firstRun) {
		firstRun = false;
		startTime = world->GetClock();
		Robot::driveBase->SetTargetAngle(angle);
		startEncoder = Robot::driveBase->GetFrontLeftDrive()->GetEncPosition();
		targetPulses = DriveUnit::ToPulses(targetDistance, units);
	}

	const int currentPosition = Robot::driveBase->GetFrontLeftDrive()->GetEncPosition();
	const double elapsedTime = world->GetClock() - startTime;
	const double currentError = fabs(currentPosition - startEncoder);

	cout << "SimpleEncoderDrive Encoder       : " << currentPosition << "\n";
	cout << "SimpleEncoderDrive Target        : " << targetPulses << "\n";
	cout << "PIDControlledDrive Current Error : " << currentError << "\n";


	if (currentError >= targetPulses) {
		cout << "Position reached in " << elapsedTime << "\n";
		crab->Stop();
		return true;
	} else if (elapsedTime > timeout) {
		cout << "***** Timed out *****";
		crab->Stop();
		return true;
	} else {
		crab->Update(Robot::driveBase->GetTwistControlOutput(), ySpeed, xSpeed, true);
		return false;
	}
}



bool PIDControlledDrive::Run(std::shared_ptr<World> world) {
	if (startTime < 0) {
		targetSetpoint = DriveUnit::ToPulses(targetDistance, units);
		startTime = world->GetClock();
		Robot::driveBase->SetTargetAngle(angle);
		Robot::driveBase->SetTargetDriveDistance(targetSetpoint, speed);
		Robot::driveBase->UseClosedLoopDrive();
		startingEncoderCount = Robot::driveBase->GetDriveControlEncoderPosition();
	}

	const double currentEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	const double currentError = Robot::driveBase->GetDriveControlError();
	const double elapsedTimeMillis = world->GetClock() - startTime;
	const double currentPIDOutput = Robot::driveBase->GetDriveControlOutput();

	SmartDashboard::PutNumber("PIDController Output", currentPIDOutput);
	cout << "PIDControlledDrive target setpoint          = " << targetSetpoint << '\n';
	cout << "PIDControlledDrive Current Encoder Position = " << currentEncoderPosition << "\n";
	cout << "PIDControlledDrive Current Error            = " << currentError << "\n";
	cout << "PIDControlledDrive PID Output:              " << currentPIDOutput << "\n";

	if (abs(currentError) <= DriveUnit::ToPulses(distanceThreshold, units)) {
		cout << "!!!Position reached in " << elapsedTimeMillis << "\n";
		crab->Stop();
		return true;
	} else if (elapsedTimeMillis > 8) {
		cerr << "**** EMERGENCY HALT ***" << "\n";
		crab->Stop();
		return true;
	} else if (collisionDetector->Detect()) {
		cerr << "**** EMERGENCY HALT DUE TO COLLISION ****\n";
		crab->Stop();
		return true;
	} else {
		const double crabSpeed = currentPIDOutput * ((reverse) ? -1.0 : 1.0);
		const double xspeed = crabSpeed * sin(angle * M_PI / 180.0);
		const double yspeed = crabSpeed * cos(angle * M_PI / 180.0);
		crab->Update(
				Robot::driveBase->GetTwistControlOutput(),
				yspeed,
				xspeed,
				true);
		return false;
	}
}



bool XYPIDControlledDrive::Run(std::shared_ptr<World> world) {
	if (startTime < 0) {
		const double hypotenuse = sqrt(XtargetDistance*XtargetDistance + YtargetDistance*YtargetDistance);
		targetSetpoint = DriveUnit::ToPulses(hypotenuse, units);
		startTime = world->GetClock();
		const double targetAngle = (!useCurrentAngle) ? angle : RobotMap::gyro->GetYaw();
		Robot::driveBase->SetTargetAngle(targetAngle);

		Robot::driveBase->SetTargetDriveDistance(targetSetpoint, speed);
		Robot::driveBase->UseClosedLoopDrive();
		startingEncoderCount = Robot::driveBase->GetDriveControlEncoderPosition();
	}

	const double currentEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	const double currentError = Robot::driveBase->GetDriveControlError();
	const double elapsedTimeMillis = world->GetClock() - startTime;
	const double currentPIDOutput = Robot::driveBase->GetDriveControlOutput();

	SmartDashboard::PutNumber("PIDController Output", currentPIDOutput);
	cout << "XYPIDControlledDrive target setpoint          = " << targetSetpoint << '\n';
	cout << "XYPIDControlledDrive Current Encoder Position = " << currentEncoderPosition << "\n";
	cout << "XYPIDControlledDrive Current Error            = " << currentError << "\n";
	cout << "XYPIDControlledDrive PID Output:              " << currentPIDOutput << "\n";

	frc::SmartDashboard::PutNumber("DriveControl P", Robot::driveBase->driveControlSpeedController->GetP());

	if (distanceThreshold == -1 ) {
		if (currentEncoderPosition > targetSetpoint) {
			std::cout << "Current encoder passed target\n";
			return true;
		}
	} else if (abs(currentError) <= DriveUnit::ToPulses(distanceThreshold, units)) {
		if (thresholdCounter++ >= thresholdCounterTarget) {
			cout << "!!!Position reached in " << elapsedTimeMillis << "\n";
			crab->Stop();
			return true;
		}
	} else {
		thresholdCounter = 0;
	}

	if (elapsedTimeMillis > timeoutCommand) {
		cout << "**** EMERGENCY EXIT OF STEP DUE TO TIMEOUT ***" << "\n";
		crab->Stop();
		return true;
//	} else if (collisionDetector->Detect()) {
//		cerr << "**** EMERGENCY HALT DUE TO COLLISION ****\n";
//		crab->Stop();
//		return true;
	} else {
		const double crabSpeed = currentPIDOutput * ((reverse) ? -1.0 : 1.0);
		const double angleRadians = atan2(XtargetDistance, YtargetDistance);
		const double xspeed = crabSpeed * sin(angleRadians);
		const double yspeed = crabSpeed * cos(angleRadians);

		std::cout << "XYPIDController crabSpeed = " << crabSpeed << "\n";
		std::cout << "XYPIDController angleRads = " << angleRadians << "\n";
//		std::cout << "XYPIDController xspeed    = " << xspeed << "\n";
//		std::cout << "XYPIDController yspeed    = " << yspeed << "\n";


		crab->Update(
				Robot::driveBase->GetTwistControlOutput(),
				yspeed,
				xspeed,
				useGyro);
		return false;
	}
}



