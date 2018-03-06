#include <Subsystems/StatusReporter.h>
#include <iostream>

static uint8_t map(double x, double in_min, double in_max, uint8_t out_min, uint8_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


StatusReporter::StatusReporter(int deviceAddress) {
	// TODO Auto-generated constructor stub
	i2c.reset(new I2C(I2C::kOnboard, deviceAddress));
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
	const int DATA_SIZE = 8;
	uint8_t data[DATA_SIZE];
	data[0] = DriverStation::Alliance::kRed == DriverStation::GetInstance().GetAlliance();
	data[1] = map(DriverStation::GetInstance().GetMatchTime(), 0, 135, 135, 0);

	i2c->WriteBulk(data, DATA_SIZE);
}


