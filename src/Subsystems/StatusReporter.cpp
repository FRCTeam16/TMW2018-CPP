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
	char data[13];
	data[0] =  254;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 1;
	data[8] = 1;
	data [9] = Robot::intake->IsPickupTriggered();
	//data [10] = Robot::driveBase->();
	data [11] = DriverStation::Alliance::kRed == DriverStation::GetInstance().GetAlliance();
	data [12] = DriverStation::GetInstance().IsDSAttached();

	serial-> Write(data, 13);
	serial-> Flush();
}
