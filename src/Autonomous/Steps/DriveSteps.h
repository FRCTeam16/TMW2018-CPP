/*
 * PIDDrive.h
 */

#ifndef SRC_AUTONOMOUS_PIDDRIVE_H_
#define SRC_AUTONOMOUS_PIDDRIVE_H_

#include "WPILib.h"
#include <Autonomous/Step.h>
#include <Util/CollisionDetector.h>
#include <Util/BSGyro.h>
#include <RobotMap.h>



class TimedDrive : public Step {
public:
	TimedDrive(double _angle, double _yspeed, double _xspeed, double _timeToDrive) :
		angle(_angle), ySpeed(_yspeed), xSpeed(_xspeed), timeToDrive(_timeToDrive) {}
	bool Run(std::shared_ptr<World> world) override;
private:
	const double angle;
	const double ySpeed;
	const double xSpeed;
	const double timeToDrive;

	double startTime = -1;
};


class DriveToBump : public Step {
public:
	DriveToBump(double _angle, double _yspeed, double _xspeed, double _maxTimeToDrive, double _ignoreTime = 0.0, double _collisionThreshold = 1.0) :
		angle(_angle), ySpeed(_yspeed), xSpeed(_xspeed), maxTimeToDrive(_maxTimeToDrive), ignoreTime(_ignoreTime),
		collisionDetector(new CollisionDetector(RobotMap::gyro, _collisionThreshold)) {}
	bool Run(std::shared_ptr<World> world) override;
private:
	const double angle;
	const double ySpeed;
	const double xSpeed;
	const double maxTimeToDrive;
	const double ignoreTime;
	const std::unique_ptr<CollisionDetector> collisionDetector;

	double startTime = -1;
};



/**
 * Uses encoder counting to drive to a specified target distance
 */
class SimpleEncoderDrive : public Step {
public:
	SimpleEncoderDrive(double _angle, double _yspeed, double _xspeed, double _targetDistance, DriveUnit::Units _units, double _timeout = 2.0) :
		angle(_angle), ySpeed(_yspeed), xSpeed(_xspeed), targetDistance(_targetDistance), units(_units),
		timeout(_timeout) {}
	bool Run(std::shared_ptr<World> world) override;
private:
	bool firstRun = true;
	double startTime = -1;
	const double angle;
	const double ySpeed;
	const double xSpeed;
	const double targetDistance;
	const DriveUnit::Units units;
	double targetPulses = 0;
	double startEncoder = 0;
	const double timeout;
};


/**
 * Sets target distance as PID setpoint to drive towards.  Distances must be positive
 */
class PIDControlledDrive : public Step {
public:
	PIDControlledDrive(double _angle, double _speed, double _targetDistance,
			int _threshold, DriveUnit::Units _units, bool _reverse = false) :
			angle(_angle), speed(_speed), targetDistance(_targetDistance), distanceThreshold(
					_threshold), units(_units), reverse(_reverse),
					collisionDetector(new CollisionDetector(RobotMap::gyro, 1.0)) {}
	bool Run(std::shared_ptr<World> world) override;
private:
	double startTime = -1;
	const double angle;
	const double speed;
	const double targetDistance;
	const double distanceThreshold;
	const DriveUnit::Units units;
	const bool reverse;
	const std::unique_ptr<CollisionDetector> collisionDetector ;

	double startingEncoderCount = 0;
	double targetSetpoint = 0;
};

/*
// TODO:
class XYSimpleEncoderDriver : public Step {
	XYSimpleEncoderDriver(double _angle, double _XtargetDistance, double _YTargetDistance, DriveUnit::Units _units) :
		angle(_angle), XTargetDistance(_XtargetDistance), YTargetDistance(_YTargetDistance), units(_units) {}
	bool Run(std::shared_ptr<World> world) override;
private:
	bool firstRun = true;
	double startTime = -1;
	const double angle;
	const double XTargetDistance;
	const double YTargetDistance;
	const DriveUnit::Units units;
	double targetPulses = 0;
};
*/


/**
 * Threshold of -1 will use past the encoder target stopping
 */
class XYPIDControlledDrive : public Step {
public:
	XYPIDControlledDrive(double _angle, double _speed,
			double _XtargetDistance, double _YtargetDistance,
			double _threshold, DriveUnit::Units _units, bool _reverse = false,
			double _timeoutCommand = 6.0, bool _useGyro = true) :
		angle(_angle), speed(_speed), XtargetDistance(_XtargetDistance),
		YtargetDistance(_YtargetDistance), distanceThreshold(_threshold),
		units(_units), reverse(_reverse), timeoutCommand(_timeoutCommand),
		useGyro(_useGyro), collisionDetector(new CollisionDetector(RobotMap::gyro, 1.0)) {}

	XYPIDControlledDrive(double _angle, double _speed,
			double _XtargetDistance, double _YtargetDistance,
			double _threshold, DriveUnit::Units _units, double _collisionThreshold,
			bool _reverse = false, double _timeoutCommand = 6.0,  bool _useGyro = true) :
		angle(_angle), speed(_speed), XtargetDistance(_XtargetDistance),
		YtargetDistance(_YtargetDistance), distanceThreshold(_threshold),
		units(_units), reverse(_reverse), timeoutCommand(_timeoutCommand),
		useGyro(_useGyro), collisionDetector(new CollisionDetector(RobotMap::gyro, _collisionThreshold)) {}


	bool Run(std::shared_ptr<World> world) override;
	void SetUseCurrentAngle() { useCurrentAngle = true; }
private:
	double startTime = -1;
	bool useCurrentAngle = false;
	const double angle;
	const double speed;
	const double XtargetDistance;
	const double YtargetDistance;
	const double distanceThreshold;
	const DriveUnit::Units units;
	const bool reverse;
	const double timeoutCommand;
	const bool useGyro;
	const std::unique_ptr<CollisionDetector> collisionDetector;
	int thresholdCounter = 0;
	const int thresholdCounterTarget = 5;

	double startingEncoderCount = 0;
	double targetSetpoint = 0;
};

#endif /* SRC_AUTONOMOUS_PIDDRIVE_H_ */
