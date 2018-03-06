
#include "StatusReporter.h"
#include "WPILib.h"
#include "RobotMap.h"
#include "Robot.h"

static uint8_t map(float x, float in_min, float in_max, uint8_t out_min, uint8_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


StatusReporter::StatusReporter(int portNumber) {
	i2c.reset(new I2C(frc::I2C::Port::kOnboard, portNumber));
}

void StatusReporter::Run() {
	std::cout << "StatusReporter::Run() started\n";
	frc::SetCurrentThreadPriority(true, 10);

	while (true) {
		try {
			SendData();
		} catch (...) {
			std::cerr << "StatusReporterdefault exception handler\n";
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

	}
}

void StatusReporter::SendData() {
	// http://wpilib.screenstepslive.com/s/3120/m/7912/l/175524-sending-data-from-the-crio-to-an-arduino
		// https://www.chiefdelphi.com/forums/showthread.php?t=132572
		// 4x wheel speed
		// 4x wheel amps
		// 1 gear captured?
		// 1 time remaining

		const double maxSpeed = 13000;
		const double maxCurrent = 40;
		const uint8_t MAX = 255;

		const int DATA_SIZE = 2;
		uint8_t data[DATA_SIZE];
		int i = 0;
//		data[i++] = map(abs(RobotMap::driveBaseFrontLeftDrive->GetSpeed()), 0, maxSpeed, 0, 255);
//		data[i++] = map(RobotMap::driveBaseFrontLeftDrive->GetOutputCurrent(), 0, maxCurrent, 0, 255);
//
//		data[i++] = map(abs(RobotMap::driveBaseFrontRightDrive->GetSpeed()), 0, maxSpeed, 0, 255);
//		data[i++] = map(RobotMap::driveBaseFrontRightDrive->GetOutputCurrent(), 0, maxCurrent, 0, 255);
//
//		data[i++] = map(abs(RobotMap::driveBaseRearLeftDrive->GetSpeed()), 0, maxSpeed, 0, 255);
//		data[i++] = map(RobotMap::driveBaseRearLeftDrive->GetOutputCurrent(), 0, maxCurrent, 0, 255);
//
//		data[i++] = map(abs(RobotMap::driveBaseRearLeftDrive->GetSpeed()), 0, maxSpeed, 0, 255);
//		data[i++] = map(RobotMap::driveBaseRearLeftDrive->GetOutputCurrent(), 0, maxCurrent, 0, 255);

		data[i++] = map(Robot::intake->IsPickupTriggered(), 0, 1, 0, 255);
		if (!DriverStation::GetInstance().IsAutonomous()) {
			data[i++] = map(DriverStation::GetInstance().GetMatchTime(), 135, 0, 135, 0);
		} else {
			data[i++] = MAX;
		}

		// Debug
	//	for (int i=0; i < DATA_SIZE; i++) {
	//		std::cout << "data[" << i << "] = " << data[i] << "\n";
	//	}

		std::cout << "Starting transaction\n";
		i2c->Transaction(data, DATA_SIZE, nullptr, 0);
		std::cout << "Transaction finished\n";
}
