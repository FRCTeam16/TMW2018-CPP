// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVEBASE_H
#define DRIVEBASE_H
#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "Drive/CrabSpeed.h"
#include "Drive/DriveEncoderPIDSource.h"
#include "Drive/FrontTwoAveragingDriveEncoderPIDSource.h"
#include "Util/DriveInfo.h"

struct Wheelbase {
	double W = 0;
	double X = 0;
	double Y = 0;
};

class DriveBase: public Subsystem {
private:

	std::shared_ptr<WPI_TalonSRX> frontLeftSteer;
	std::shared_ptr<WPI_TalonSRX> frontRightSteer;
	std::shared_ptr<WPI_TalonSRX> rearLeftSteer;
	std::shared_ptr<WPI_TalonSRX> rearRightSteer;
	std::shared_ptr<WPI_TalonSRX> frontLeftDrive;
	std::shared_ptr<WPI_TalonSRX> frontRightDrive;
	std::shared_ptr<WPI_TalonSRX> rearLeftDrive;
	std::shared_ptr<WPI_TalonSRX> rearRightDrive;

	std::unique_ptr<CrabSpeed> crabSpeedTwist;
	std::unique_ptr<FrontTwoAveragingDriveEncoderPIDSource> driveControlEncoderSource;



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	DriveInfo<double> positionOffsets;
	DriveInfo<int> turns;
	DriveInfo<int> inv;
	DriveInfo<int> hotCount;

	double A = 0;					// steer mode
	double thetaRC = 0;
	DriveInfo<double> theta;		// steer mode theta
	DriveInfo<double> steerSpeed;	// steer mode wheel speeds

	int coolCount = 0;
	double driveLimit = 1.0;

	Wheelbase wheelbase;
	std::unique_ptr<PIDController> driveControlTwist;
	std::unique_ptr<CrabSpeed> driveControlDistanceSpeed;

	const int MaxTurns = 1000;
	bool driveFront = true;

	void InitializePIDs();
	void InitializeOffsets();

	void SetSteerSetpoint(double setpoint,
			std::shared_ptr<AnalogInput> actual, double offset,
			std::shared_ptr<PIDController> PIDCon, std::shared_ptr<WPI_TalonSRX> steer,
			int turns, int &inv);

	//created for new talon positioning
	double GetTalonCurrentPosition(std::shared_ptr<WPI_TalonSRX> talon);
	int GetTalonCurrentTurns(std::shared_ptr<WPI_TalonSRX> talon);
	double GetTalonCurrentOffsetPosition(std::shared_ptr<WPI_TalonSRX> talon);
	void SetTalonSteerSetpoint(double setpoint,
			double offset,	std::shared_ptr<WPI_TalonSRX> steer,  int &inv);


	double CorrectSteerSetpoint(double setpoint);
	void SetDriveSpeed(DriveInfo<double> speed);

public:
	DriveBase();
	void InitDefaultCommand();
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	DriveInfo<double> CalculatePositionOffsets();
	void SetPositionOffsets(DriveInfo<double> &info);
	DriveInfo<double> GetPositionOffsets() const;
	void ZeroTurnInfo();
	void ZeroDriveEncoders();
	void SetSteering(DriveInfo<double> setpoint);

	void InitTeleop();
	void InitAuto();

	void Lock();
	void Crab(double twist, double y, double x, bool useGyro);

	void SetTargetAngle(double angle);
	double GetTwistControlOutput();
	double GetTwistControlError();

	void SetTargetDriveDistance(double distance, double maxSpeed = 0.5);
	void UseClosedLoopDrive();
	void HaltUsingClosedLoop();
	void UseOpenLoopDrive();

	double GetDriveControlEncoderPosition();
	double GetDriveControlOutput();
	double GetDriveControlError();
	double GetDriveControlP();

	Wheelbase GetWheelbase();

	void SetConstantVelocity(double twistInput, double velocity);

	void Instrument();

	std::unique_ptr<PIDController> driveControlSpeedController;

};
#endif
