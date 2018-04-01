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


inline void add(DriveInfo<double> &di1, const DriveInfo<int> &di2) {
	di1.FL += abs(di2.FL);
	di1.FR += abs(di2.FR);
	di1.RL += abs(di2.RL);
	di1.RR += abs(di2.RR);
}

inline void add(DriveInfo<double> &di1, const DriveInfo<double> &di2) {
	di1.FL += fabs(di2.FL);
	di1.FR += fabs(di2.FR);
	di1.RL += fabs(di2.RL);
	di1.RR += fabs(di2.RR);
}

inline double CalculateAverage(const DriveInfo<double> &data) {
	double sum = data.FL + data.FR + data.RL + data.RR;
	return sum / 4.0;
}

inline double CalculateAverage(const DriveInfo<int> &data) {
	double sum = data.FL + data.FR + data.RL + data.RR;
	return sum / 4.0;
}



void DmsProcessManager::SetRunning(bool _run) {
	if (running && !_run) {
		Reset();
	} else if (!running && _run) {
		currentPhase = kTestDriveMotors;
	}
	running = _run;
}

void DmsProcessManager::Run() {
	statusReporter->SetDmsMode(running);
	if (running) {
		switch(currentPhase) {
		case TestPhase::kStopped:
			break;
		case kTestDriveMotors:
			DoMotorTest();
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
	}
	const double elapsedTime = (currentTime - startTime);

	if (elapsedTime < motorTestTime) {
		add(driveCurrent, Robot::driveBase->GetSteerCurrent());
		add(driveEncoder, Robot::driveBase->GetDriveEncoderPositions());
		loopCounter++;
		Robot::driveBase->SetConstantVelocity(0.0, 1.0);

		std::cout << "(Enc) FL: " << driveEncoder.FL
				  << " FR: " << driveEncoder.FR
				  << " RL: " << driveEncoder.RL
				  << " RR: " << driveEncoder.RR
				  << "\n";

		std::cout << "(Amp) FL: " << driveCurrent.FL
				  << " FR: " << driveCurrent.FR
				  << " RL: " << driveCurrent.RL
				  << " RR: " << driveCurrent.RR
				  << "\n";

		const double encAvg = CalculateAverage(driveEncoder);
		const double ampAvg = CalculateAverage(driveCurrent);
		std::cout << "Enc Avg: " << encAvg << " | Amp Avg: " << ampAvg << "\n";
		DriveInfo<int> status;
		status.FL = CalculateStatus(driveEncoder.FL, encAvg, driveCurrent.FL, ampAvg);
		status.FR = CalculateStatus(driveEncoder.FR, encAvg, driveCurrent.FR, ampAvg);
		status.RL = CalculateStatus(driveEncoder.RL, encAvg, driveCurrent.RL, ampAvg);
		status.RR = CalculateStatus(driveEncoder.RR, encAvg, driveCurrent.RR, ampAvg);

		statusReporter->SetDriveStatus(status);

		std::cout << "FL: " << status.FL
				  << " FR: " << status.FR
				  << " RL: " << status.RL
				  << " RR: " << status.RR
				  << "\n";

		Robot::driveBase->SetConstantVelocity(0.0, 1.0);
	} else {
		Robot::driveBase->SetConstantVelocity(0.0, 0.0);
	}
}

int DmsProcessManager::CalculateStatus(const double enc, const double encAvg, const double amp, const double ampAvg) {
	if (amp == 0) {
		return 4;	// No Amps
	}
	if (enc == 0) {
		return 5;	// No encoder counts
	}
	bool encOutside = (enc / encAvg) < encAvgThreshold;
	bool ampOutside = (amp / ampAvg) < ampAvgThreshold;
	if (!encOutside && !ampOutside) {
		return 1;	// Good
	} else if (!encOutside && ampOutside) {
		return 2;	// Amp outside, enc ok
	} else if (encOutside && !ampOutside) {
		return 3;	// Enc outside of value, amp is ok
	} else {
		// Both enc & amp outside of ranges
		return 2; 	// both outside, show amp
	}
}

void DmsProcessManager::Reset() {
	currentPhase = kStopped;
	startTime = -1;
	loopCounter = 0;
	driveCurrent = DriveInfo<double>(0.0);
	driveEncoder = DriveInfo<double>(0.0);
}
