#include <Subsystems/StatusReporter.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <Robot.h>

using namespace std;

namespace StatusReporterUtil {
	static uint8_t map(double x, double in_min, double in_max, uint8_t out_min, uint8_t out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}

void StatusReporter::Launch() {
	std::cout << "StatusReporter::Launch...";
	reporterThread = std::thread(&StatusReporter::Run, this);
	reporterThread.detach();
	std::cout << "done\n";
}

StatusReporter::StatusReporter() {
	serial.reset(
		new SerialPort(
			9600,
			SerialPort::Port::kOnboard,
			8,
			SerialPort::Parity::kParity_None,
			SerialPort::StopBits::kStopBits_One));
	if (serial.get()) {
		running = true;
	} else {
		std::cout << "!!! Unable to start serial communications !!!\n";
	}

}


void StatusReporter::Run() {
	frc::SetCurrentThreadPriority(true, 10);

	while (running) {
		try {
			SendData();
		} catch(...) {
			std::cout << "StatusReporter exited\n";
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::cout << "StatusReporter::Run exited\n";
}

void StatusReporter::SendData() {
	stringstream ss;
	const bool isRed = DriverStation::Alliance::kRed == DriverStation::GetInstance().GetAlliance();

	DriveInfo<int> driveEncoders = Robot::driveBase->GetDriveControlEncoderPosition();
	DriveInfo<double> driveCurrents = Robot::driveBase->GetDriveCurrent();
	DriveInfo<int> steerEncoders = Robot::driveBase->GetSteerEncoderPositions();
	DriveInfo<double> steerCurrents = Robot::driveBase->GetSteerCurrent();


	// TODO: Evaluate drive currents and send whether it is good or bad 0-4
	// 10% threshold range | Example - 0 (red) 1 - 9, 11 - ? (orange), 9-11 (green)
	// Evaluate encoders via averaging dropout

	// 1-4 solid, 5-7 flashing

	// alliance color
	// have cube?
	// disabled, auto, teleop
	// climb sequence #s (ClimbState)
	// 4 drive motor currents (evalution?)

	ss << ((isRed) ? 0 : 1) << delimiter;
	ss << StatusReporterUtil::map(DriverStation::GetInstance().GetMatchTime(), 0, 135, 135, 0) << delimiter;


	ss << driveEncoders.FL << delimiter;
	ss << driveEncoders.FR << delimiter;
	ss << driveEncoders.RR << delimiter;
	ss << driveEncoders.RL << delimiter;

	ss << driveCurrents.FL << delimiter;
	ss << driveCurrents.FR << delimiter;
	ss << driveCurrents.RR << delimiter;
	ss << driveCurrents.RL << delimiter;

	ss << steerEncoders.FL << delimiter;
	ss << steerEncoders.FR << delimiter;
	ss << steerEncoders.RR << delimiter;
	ss << steerEncoders.RL << delimiter;

	ss << steerCurrents.FL << delimiter;
	ss << steerCurrents.FR << delimiter;
	ss << steerCurrents.RR << delimiter;
	ss << steerCurrents.RL << delimiter;



	serial->Write(ss.str());
}
