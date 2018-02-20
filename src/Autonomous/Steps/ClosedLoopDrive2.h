
#ifndef SRC_AUTONOMOUS_STEPS_CLOSEDLOOPDRIVE2_H_
#define SRC_AUTONOMOUS_STEPS_CLOSEDLOOPDRIVE2_H_

#include <Autonomous/Step.h>
#include <Autonomous/DriveUnit.h>
#include <Util/CollisionDetector.h>

class ClosedLoopDrive2: public Step {
public:
	ClosedLoopDrive2(double _angle, double _speed, double _x, double _y,
					double _threshold, DriveUnit::Units _units,
					double _timeout,
					double _rampUp, double _rampDown) :
		angle(_angle),
		speed(_speed),
		XtargetDistance(_x),
		YtargetDistance(_y),
		distanceThreshold(_threshold),
		units(_units),
		timeoutCommand(_timeout),
		rampUp(_rampUp),
		rampDown(_rampDown),
		collisionDetector(CollisionDetector(RobotMap::gyro, 1.0))
	{
	}

	virtual ~ClosedLoopDrive2();
	bool Run(std::shared_ptr<World> world) override;
	void setUseCurrentAngle();

private:
	const double angle;
	const double speed;
	const double XtargetDistance;
	const double YtargetDistance;
	const double distanceThreshold;     // -1 value simply checks for passing the setpoint
	const DriveUnit::Units units;
	const double timeoutCommand = 10;
	const double rampUp;	// ramp up time length, -1 to disable
	const double rampDown;	// ramp down position threshold, -1 to disable
	CollisionDetector collisionDetector;
	const bool reverse = false;
	const bool useGyro = true;
	const int thresholdCounterTarget = 5;


	double startTime = -1;
	bool useCurrentAngle = false;
	int thresholdCounter = 0;
	double targetSetpoint = 0;
};

#endif /* SRC_AUTONOMOUS_STEPS_CLOSEDLOOPDRIVE2_H_ */
