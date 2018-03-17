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


StatusReporter::StatusReporter() {
	serial.reset(
		new SerialPort(
			9600,
			SerialPort::Port::kOnboard,
			8,
			SerialPort::Parity::kParity_None,
			SerialPort::StopBits::kStopBits_One));

}


void StatusReporter::Run() {
	frc::SetCurrentThreadPriority(true, 10);

	while (true) {
		try {
			SendData();
		} catch(...) {
			std::cout << "StatusReporter exited\n";
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void StatusReporter::SendData() {
	stringstream ss;
	const bool isRed = DriverStation::Alliance::kRed == DriverStation::GetInstance().GetAlliance();
	ss << ((isRed) ? 0 : 1) << delimiter;
	ss << StatusReporterUtil::map(DriverStation::GetInstance().GetMatchTime(), 0, 135, 135, 0) << delimiter;

	DriveInfo<int> driveEncoders = Robot::driveBase->GetDriveControlEncoderPosition();
	DriveInfo<double> driveCurrents = Robot::driveBase->GetDriveCurrent();
	DriveInfo<int> steerEncoders = Robot::driveBase->GetSteerEncoderPositions();
	DriveInfo<double> steerCurrents = Robot::driveBase->GetSteerCurrent();

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
