#ifndef SRC_DMS_DMSPROCESSMANAGER_H_
#define SRC_DMS_DMSPROCESSMANAGER_H_

#include "WPILib.h"
#include <Subsystems/StatusReporter.h>
#include <Util/DriveInfo.h>


class DmsProcessManager {
public:
	DmsProcessManager(std::shared_ptr<StatusReporter> _statusReporter) : statusReporter(_statusReporter) {}
	virtual ~DmsProcessManager() {}
	void Run();

	void SetRunning(bool _run);
	bool IsRunning() { return running; }
private:
	enum TestPhase { kStopped, kTestDriveMotors, kTestSteerMotors };
	const std::shared_ptr<StatusReporter> statusReporter;
	double startTime = -1;
	double loopCounter = 0;
	bool running = false;
	TestPhase currentPhase = kStopped;

	DriveInfo<int> startDriveEncoder;
	DriveInfo<double> driveCurrent;
	DriveInfo<double> driveVelocity;

	DriveInfo<int> startSteerVelocity;
	DriveInfo<double> steerCurrent;
	DriveInfo<double> steerVelocity;

	const DriveInfo<int> ZERO_DI;

	const double initialIgnoreTime = 1.0;
	const double encAvgThreshold = 0.8;
	const double ampAvgThreshold = 0.8;
	const double motorTestTime = 4.0;


	void DoMotorTest();
	void DoSteerTest();

	void Reset();
	int CalculateStatus(const double enc, const double encAvg, const double amp, const double ampAvg);
};

#endif /* SRC_DMS_DMSPROCESSMANAGER_H_ */
