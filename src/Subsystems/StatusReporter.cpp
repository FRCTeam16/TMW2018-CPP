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
			115200,
			SerialPort::Port::kMXP,
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
	const double x = Robot::driveBase->GetLastSpeedX();
	const double y = Robot::driveBase->GetLastSpeedY();
	const double speed = sqrt(fabs(x*x) + fabs(y*y));

//	std::cout << "Sending DMS ? " << dmsMode << "\n";

	const int DATA_SIZE = 14;
	char data[DATA_SIZE];
	data[0]  = (char) 254;
	data[1]  = (char) (dmsMode) ? driveStatus.FL : 0;
	data[2]  = (char) (dmsMode) ? steerStatus.FL : 0;
	data[3]  = (char) (dmsMode) ? driveStatus.FR : 0;
	data[4]  = (char) (dmsMode) ? steerStatus.FR : 0;
	data[5]  = (char) (dmsMode) ? driveStatus.RL : 0;
	data[6]  = (char) (dmsMode) ? steerStatus.RL : 0;
	data[7]  = (char) (dmsMode) ? driveStatus.RR : 0;
	data[8]  = (char) (dmsMode) ? steerStatus.RR : 0;
	data[9]  = (char) Robot::intake->IsPickupTriggered();
	data[10] = (char) StatusReporterUtil::map(speed, 0.0, 1.0, 0, 250);
	data[11] = (char) DriverStation::Alliance::kRed == DriverStation::GetInstance().GetAlliance();
	data[12] = (char) 1; // DriverStation::GetInstance().IsDSAttached();
	data[13] = (char) StatusReporterUtil::map(Robot::elevator->GetElevatorEncoderPosition(), 0, 59000, 0, 250);

	serial->Write(data, DATA_SIZE);
	serial->Flush();
}
