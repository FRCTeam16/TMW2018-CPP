#include <DMS/DmsProcessManager.h>

#include <Robot.h>

/**
0 - no updated data (purple)
1 - both amperage draw and encoder values were as expected (green light)
2 - amperage was outside (high or lower, but not 0) than expected, encoder value was expected (yellow/flashing)
3 - amperage as expected, encoder value was outside of expected range (but not 0) (yellow/solid)
4 - amperage draw was 0 (red/flashing)
5 - encoder value was 0 (red
 */


inline void add(DriveInfo<int> &di1, const DriveInfo<int> &di2) {
	di1.FL += di2.FL;
	di1.FR += di2.FR;
	di1.RL += di2.RL;
	di1.RR += di2.RR;
}

inline void add(DriveInfo<double> &di1, const DriveInfo<double> &di2) {
	di1.FL += di2.FL;
	di1.FR += di2.FR;
	di1.RL += di2.RL;
	di1.RR += di2.RR;
}


void DmsProcessManager::Run() {
	if (running) {
		switch(currentPhase) {
		case kTestDriveMotors:
			break;
		case kTestSteerMotors:
			break;
		}
	}
}

void DmsProcessManager::DoMotorTest() {
	const double currentTime = Timer::GetFPGATimestamp();
	if (startTime < 0) {
		startTime = currentTime;
		loopCounter = 0;
		driveCurrent = DriveInfo<double>(0.0);
		driveEncoder = DriveInfo<int>(0);
	}
	const double elapsedTime = (currentTime - startTime);
	if (elapsedTime < motorTestTime) {
		add(driveCurrent, Robot::driveBase->GetSteerCurrent());
		add(driveEncoder, Robot::driveBase->GetDriveEncoderPositions());
		loopCounter++;
	} else {

	}

}


