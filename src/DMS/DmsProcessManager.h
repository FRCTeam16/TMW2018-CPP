#ifndef SRC_DMS_DMSPROCESSMANAGER_H_
#define SRC_DMS_DMSPROCESSMANAGER_H_

#include "WPILib.h"
#include <Robot.h>


class DmsProcessManager {
public:
	DmsProcessManager() {}
	virtual ~DmsProcessManager() {}
	void Run();

	void SetRunning(bool _run) { running = _run; }
	bool IsRunning() { return running; }
private:
	enum TestPhase { kStopped, kTestDriveMotors, kTestSteerMotors };
	double startTime = -1;
	double loopCounter = 0;
	bool running = false;
	TestPhase currentPhase;

	DriveInfo<double> driveCurrent;
	DriveInfo<int> driveEncoder;

	const double motorTestTime = 3.0;

	void DoMotorTest();
	void Reset();
};

#endif /* SRC_DMS_DMSPROCESSMANAGER_H_ */
