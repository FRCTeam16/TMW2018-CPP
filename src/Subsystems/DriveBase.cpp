// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include <iostream>
#include <vector>

#include "ctre/Phoenix.h"
#include "DriveBase.h"
#include "../RobotMap.h"
#include "Drive/CrabSpeed.h"
#include "Drive/DriveEncoderPIDSource.h"
#include "../Util/PrefUtil.h"



//#define DEBUG 1

#ifdef DEBUG
#define D(x) x
#define DM(x) (std::cout << x << std::endl)
#else
#define D(x)
#define DM(x)
#endif


using namespace frc;

DriveBase::DriveBase() : Subsystem("DriveBase") {
	std::cout << "DriveBase::DriveBase()\n";

	frontLeftSteer = RobotMap::driveBaseFrontLeftSteer;
	frontRightSteer = RobotMap::driveBaseFrontRightSteer;
	rearLeftSteer = RobotMap::driveBaseRearLeftSteer;
	rearRightSteer = RobotMap::driveBaseRearRightSteer;

	frontLeftDrive = RobotMap::driveBaseFrontLeftDrive;
	frontRightDrive = RobotMap::driveBaseFrontRightDrive;
	rearLeftDrive = RobotMap::driveBaseRearLeftDrive;
	rearRightDrive = RobotMap::driveBaseRearRightDrive;

    crabSpeedTwist.reset(new CrabSpeed());

	InitializeOffsets();

    wheelbase.W = 23.5/2;
    wheelbase.X = 27.5;
    wheelbase.Y = 23.5/2;

    // Initialize Steering
    const std::vector<std::shared_ptr<WPI_TalonSRX>> steering {
    	frontLeftSteer, frontRightSteer, rearLeftSteer, rearRightSteer };
    std::for_each(steering.begin(), steering.end(), [](std::shared_ptr<WPI_TalonSRX> steer) {
    	steer->SetNeutralMode(NeutralMode::Coast);
    	steer->Set(ControlMode::Position, 0);
    	steer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
    	steer->SetInverted(false);
    	steer->Config_kP(0, 1.0, 0);
    	steer->ConfigPeakOutputForward(0.75, 0);
    	steer->ConfigPeakOutputReverse(-0.75, 0);
    });
    InitializePIDs();
	std::cout << "DriveBase::DriveBase() Finished\n";
}

void DriveBase::InitializePIDs() {
	 // Initialize Drive Talons + PID feedback
	Preferences *prefs = Preferences::GetInstance();
	const double izone = PrefUtil::getSet("DriveControlTwistIZone", 0.0);

	if (driveControlTwist == nullptr) {
		driveControlTwist.reset(
					new PIDController(
							prefs->GetFloat("TwistP", 0.02),
							prefs->GetFloat("TwistI", 0.0),
							prefs->GetFloat("TwistD", 0.12),
							0.0, RobotMap::gyro.get(), crabSpeedTwist.get(), 0.02 ));
	} else {
		driveControlTwist->SetPID(
				prefs->GetFloat("TwistP", 0.02),
				prefs->GetFloat("TwistI", 0.0),
				prefs->GetFloat("TwistD", 0.12));
		// was setting izone in custom PID
	}
	driveControlTwist->SetContinuous(true);
	driveControlTwist->SetAbsoluteTolerance(2.0);
	driveControlTwist->Enable();
	driveControlTwist->SetOutputRange(-0.5, 0.5);
	driveControlTwist->SetInputRange(-180, 180);

	const double driveP = prefs->GetFloat("DriveP");
	const double driveI = prefs->GetFloat("DriveI");
	const double driveD = prefs->GetFloat("DriveD");
	const double driveF = prefs->GetFloat("DriveF");
	const double driveIZone = prefs->GetFloat("DriveIZone");

	const std::vector<std::shared_ptr<WPI_TalonSRX>> drives {
		frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive};
	std::for_each(drives.begin(), drives.end(), [&](std::shared_ptr<WPI_TalonSRX> drive) {
		drive->SetNeutralMode(NeutralMode::Coast);
		drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		drive->Set(ControlMode::Velocity, 0);

		drive->Config_kP(0, driveP, 0);
		drive->Config_kI(0, driveI, 0);
		drive->Config_kD(0, driveD, 0);
		drive->Config_kF(0, driveF, 0);
		drive->Config_IntegralZone(0,  driveIZone, 0);
	});

	// Drive PID Control
	const double driveControlP = prefs->GetFloat("DriveControlP");
	const double driveControlI = prefs->GetFloat("DriveControlI");
	const double driveControlD = prefs->GetFloat("DriveControlD");
	const double driveControlF = prefs->GetFloat("DriveControlF");
	const double driveControlIZone = prefs->GetFloat("DriveControlIZone");
	if (driveControlEncoderSource == nullptr) {
		DriveInfo<std::shared_ptr<WPI_TalonSRX>> motors;
		motors.FL = frontLeftDrive;
		motors.FR = frontRightDrive;
		motors.RL = rearLeftDrive;
		motors.RR = rearRightDrive;
		driveControlEncoderSource.reset(new FrontTwoAveragingDriveEncoderPIDSource(motors));
	}

	if (driveControlDistanceSpeed == nullptr) {
		driveControlDistanceSpeed.reset(new CrabSpeed());
	}

	if (driveControlSpeedController == nullptr) {
		driveControlSpeedController.reset(
				new PIDController(driveControlP, driveControlI, driveControlD, driveControlF,
						driveControlEncoderSource.get(),
						driveControlDistanceSpeed.get(),
						0.05));
//		driveControlSpeedController->SetIzone(driveControlIZone);
		driveControlSpeedController->Enable();
	} else {
		driveControlSpeedController->SetPID(driveControlP, driveControlI, driveControlD, driveControlF);
//		driveControlSpeedController->SetIzone(driveControlIZone);
	}
}

void DriveBase::InitDefaultCommand() {
}


// Put methods for controlling this subsystem
// here. Call these from Commands.

void DriveBase::InitializeOffsets() {
	positionOffsets.FL = PrefUtil::getSet("FLOff", 0.0);
	positionOffsets.FR = PrefUtil::getSet("FROff", 0.0);
	positionOffsets.RL = PrefUtil::getSet("RLOff", 0.0);
	positionOffsets.RR = PrefUtil::getSet("RROff", 0.0);
	std::cout << "DriveBase loaded offsets\n";
}

void DriveBase::Lock() {
	DriveInfo<double> steering;
	steering.FL = 0.375;
	steering.FR = 0.75;
	steering.RL = 0.75;
	steering.RR = 4.5;	// FIXME: Units need updating
	SetSteering(steering);

	DriveInfo<double> lockSpeed;
	SetDriveSpeed(lockSpeed);
}

DriveInfo<double> DriveBase::CalculatePositionOffsets() {
	positionOffsets.FL = GetTalonCurrentPosition(frontLeftSteer);
	positionOffsets.FR = GetTalonCurrentPosition(frontRightSteer);
	positionOffsets.RL = GetTalonCurrentPosition(rearLeftSteer);
	positionOffsets.RR = GetTalonCurrentPosition(rearRightSteer);
	return positionOffsets;
}


void DriveBase::ZeroTurnInfo() {
	turns.FL = 0;
	turns.FR = 0;
	turns.RL = 0;
	turns.RR = 0;
}

void DriveBase::ZeroDriveEncoders() {
	const std::vector<std::shared_ptr<WPI_TalonSRX>> drives {frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive};
	std::for_each(drives.begin(), drives.end(),
			[](std::shared_ptr<WPI_TalonSRX> motor) {
		motor->SetSelectedSensorPosition(0, 0, 0);
	});
}

void DriveBase::InitTeleop() {
	InitializePIDs();
	this->UseOpenLoopDrive();

	const std::vector<std::shared_ptr<WPI_TalonSRX>> steers {frontLeftSteer, frontRightSteer, rearLeftSteer, rearRightSteer};
		std::for_each(steers.begin(), steers.end(), [](std::shared_ptr<WPI_TalonSRX> steer) {
			steer->Config_kP(0, 1.0, 0);
		});
}

void DriveBase::InitAuto() {
	InitializePIDs();
	UseClosedLoopDrive();
	const std::vector<std::shared_ptr<WPI_TalonSRX>> steers {frontLeftSteer, frontRightSteer, rearLeftSteer, rearRightSteer};
	std::for_each(steers.begin(), steers.end(), [](std::shared_ptr<WPI_TalonSRX> steer) {
		steer->Config_kP(0, 3.0, 0);
	});
}

void DriveBase::UseOpenLoopDrive() {
	std::cout << "*** UseOpenLoopDrive ***\n";
	const std::vector<std::shared_ptr<WPI_TalonSRX>> drives { frontLeftDrive,
			frontRightDrive, rearLeftDrive, rearRightDrive };
	std::for_each(drives.begin(), drives.end(),
			[](std::shared_ptr<WPI_TalonSRX> motor) {
				motor->Set(ControlMode::PercentOutput, 0.0);
			});
}


void DriveBase::UseClosedLoopDrive() {
	std::cout << "*** UseClosedLoopDrive ***\n";
	const std::vector<std::shared_ptr<WPI_TalonSRX>> drives { frontLeftDrive,
				frontRightDrive, rearLeftDrive, rearRightDrive };
	std::for_each(drives.begin(), drives.end(),
			[](std::shared_ptr<WPI_TalonSRX> motor) {
				motor->Set(ControlMode::Velocity, 0);
			});
}

void DriveBase::HaltUsingClosedLoop() {
	const std::vector<std::shared_ptr<WPI_TalonSRX>> drives { frontLeftDrive,
					frontRightDrive, rearLeftDrive, rearRightDrive };
	std::for_each(drives.begin(), drives.end(),
			[](std::shared_ptr<WPI_TalonSRX> motor) {
				motor->Config_kP(0, 0.1, 0);
			});
	Crab(0.0, 0.0, 0.0, true);
}

void DriveBase::Crab(double twist, double y, double x, bool useGyro) {
	float FWD = y;
	float STR = x;

	if (useGyro) {
		const double robotangle = RobotMap::gyro->GetYaw() * M_PI / 180;
		FWD =  y * cos(robotangle) + x * sin(robotangle);
		STR = -y * sin(robotangle) + x * cos(robotangle);
	}

	const double radius = sqrt(pow(2*wheelbase.Y, 2) + pow(wheelbase.X,2));
	double AP = STR - twist * wheelbase.X / radius;
	double BP = STR + twist * wheelbase.X / radius;
	double CP = FWD - twist * 2 * wheelbase.Y / radius;
	double DP = FWD + twist * 2 * wheelbase.Y / radius;


	DriveInfo<double> setpoint(0.0);

	if (DP != 0 || BP != 0) {
		setpoint.FL = atan2(BP,DP) * (180/M_PI);
	}
	if (BP != 0 || CP != 0) {
		setpoint.FR = atan2(BP, CP) * (180/M_PI);
	}
	if (AP != 0 || DP != 0) {
		setpoint.RL = atan2(AP, DP) * (180/M_PI);
	}
	if (AP != 0 || CP != 0) {
		setpoint.RR = atan2(AP, CP) * (180/M_PI);
	}

	SetSteering(setpoint);


	DriveInfo<double> speed(0.0);
	speed.FL = sqrt(pow(BP, 2) + pow(DP, 2));
	speed.FR = sqrt(pow(BP, 2) + pow(CP, 2));
	speed.RL = sqrt(pow(AP, 2) + pow(DP, 2));
	speed.RR = sqrt(pow(AP, 2) + pow(CP, 2));

	double speedarray[] = {fabs(speed.FL), fabs(speed.FR), fabs(speed.RL), fabs(speed.RR)};
	const double maxspeed = *std::max_element(speedarray, speedarray+4);

	DriveInfo<double> ratio;
	if (maxspeed > 1 || maxspeed < -1) {
		ratio.FL = speed.FL / maxspeed;
		ratio.FR = speed.FR / maxspeed;
		ratio.RL = speed.RL / maxspeed;
		ratio.RR = speed.RR / maxspeed;
	} else {
		ratio.FL = speed.FL;
		ratio.FR = speed.FR;
		ratio.RL = speed.RL;
		ratio.RR = speed.RR;
	}
	ratio.FR = -ratio.FR;
	ratio.RR = -ratio.RR;
	SetDriveSpeed(ratio);
}

void DriveBase::SetSteering(DriveInfo<double> setpoint) {
	if (driveFront) {
		DM("FL");
		SetTalonSteerSetpoint(setpoint.FL, positionOffsets.FL, frontLeftSteer, inv.FL);
		DM("FR");
		SetTalonSteerSetpoint(setpoint.FR, positionOffsets.FR, frontRightSteer, inv.FR);
		DM("RL");
		SetTalonSteerSetpoint(setpoint.RL, positionOffsets.RL, rearLeftSteer, inv.RL);
		DM("RR");
		SetTalonSteerSetpoint(setpoint.RR, positionOffsets.RR, rearRightSteer, inv.RR);

	} else {
		SetTalonSteerSetpoint(setpoint.RR, positionOffsets.FL, frontLeftSteer, inv.FL);
		SetTalonSteerSetpoint(setpoint.RL, positionOffsets.FR, frontRightSteer, inv.FR);
		SetTalonSteerSetpoint(setpoint.FR, positionOffsets.RL, rearLeftSteer, inv.RL);
		SetTalonSteerSetpoint(setpoint.FL, positionOffsets.RR, rearRightSteer, inv.RR);
	}
}

double DriveBase::GetTalonCurrentPosition(std::shared_ptr<WPI_TalonSRX> talon) {
	int currentPosition = talon->GetSelectedSensorPosition(0);
	int currentPositionEncoderUnits = currentPosition % 4096;
	double positionInDegrees = (currentPositionEncoderUnits / 4096.0 * 360.0);

	if (positionInDegrees < -180.0) {
		positionInDegrees += 360.0;
	} else if (positionInDegrees > 180.0) {
		positionInDegrees -= 360.0;
	}
	return positionInDegrees;
}

void DriveBase::SetTalonSteerSetpoint(double setpoint, double offset, std::shared_ptr<WPI_TalonSRX> steer, int &inv){
	double currentPosition = steer->GetSelectedSensorPosition(0) / 4096.0;
	double setpointRotations = (setpoint + offset) / 360.0;

	double wholeRotations = 0.0;
	double diff = modf(setpointRotations - currentPosition, &wholeRotations);
	if (fabs(diff) > 0.25) {
		diff -= copysign(0.5, diff);
		inv = -1;
	} else {
		inv = 1;
	}

	double finalSetpoint = currentPosition + diff;
//	std::cout << "current: " << currentPosition <<
//			" | setpoint: " << setpoint <<
//			" | diff: " << diff <<
//			" | final: " << finalSetpoint << std::endl;
	steer->Set(ControlMode::Position, finalSetpoint * 4096);
}

void DriveBase::SetSteerSetpoint(double setpoint,
		std::shared_ptr<AnalogInput> actual, double offset,
		std::shared_ptr<PIDController> PIDCon, std::shared_ptr<WPI_TalonSRX> steer,
		int turns, int &inv) {
	const double volt = actual->GetVoltage();

	if (turns >= MaxTurns) {
		PIDCon->Disable();
		steer->Set(-1);
	} else if (turns <= -MaxTurns) {
		PIDCon->Disable();
		steer->Set(1);
	} else if (fabs(CorrectSteerSetpoint(setpoint + offset) - volt) < 1.25
			|| fabs(CorrectSteerSetpoint(setpoint + offset) - volt) > 3.75) {
		PIDCon->Enable();
		if ((turns + 1 == MaxTurns
				&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint) > 2.5)
				|| (turns - 1 == -MaxTurns
						&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint) < -2.5)) {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset - 2.5));
			inv = -1;
			D(std::cout << "SetSteer Inv #1 = -1" << std::endl;)
		} else {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset));
			inv = 1;
			D(std::cout << "SetSteer Inv #2 = 1" << std::endl;)
		}
	} else {
		PIDCon->Enable();
		if ((turns + 1 == MaxTurns
				&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint - 2.5) > 2.5)
				|| (turns - 1 == -MaxTurns
						&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint - 2.5) < -2.5)) {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset));
			inv = 1;
			D(std::cout << "SetSteer Inv #3 = 1" << std::endl;)
		} else {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset - 2.5));
			D(std::cout << "SetSteer Inv #4 = -1" << std::endl;)
			inv = -1;
		}
	}
}

double DriveBase::CorrectSteerSetpoint(double setpoint) {
	//Used to correct steering setpoints to within the 0 to 5 V scale
	if (setpoint < 0) {
		return setpoint + 5;
	} else if (setpoint > 5) {
		return setpoint - 5;
	} else if (setpoint == 5) {
		return 0;
	} else {
		return setpoint;
	}
}


void DriveBase::SetDriveSpeed(DriveInfo<double> speed) {
	const float driveOutLimit = 50;
	const int hotCountLimit = 100;
	const int coolCountLimit = 1000;

	frontLeftDrive->GetOutputCurrent() > driveOutLimit ? hotCount.FL++ : hotCount.FL =0;
	frontRightDrive->GetOutputCurrent() > driveOutLimit ? hotCount.FR++ : hotCount.FR =0;
	rearLeftDrive->GetOutputCurrent() > driveOutLimit ? hotCount.RL++ : hotCount.RL =0;
	rearRightDrive->GetOutputCurrent() > driveOutLimit ? hotCount.RR++ : hotCount.RR =0;

	if (hotCount.FL == 0 && hotCount.FR == 0 && hotCount.RL == 0 && hotCount.RR == 0 ) {
		coolCount++;
	} else {
		coolCount = 0;
	}

	if (hotCount.FL > hotCountLimit && hotCount.FR > hotCountLimit &&
			hotCount.RL > hotCountLimit  && hotCount.RR > hotCountLimit) {
		driveLimit = 0.5;
	}

	if (coolCount > coolCountLimit) {
		driveLimit = 1.0;
	}

	// Check one of our drives to see if the mode is open or closed loop
	// FIXME: RPM Scale factor needs to be in preferences
	const ControlMode controlMode = frontLeftDrive->GetControlMode();
	const float SCALE_FACTOR = (ControlMode::PercentOutput == controlMode) ? 1 : 13000;

	if(driveFront) {
		frontLeftDrive-> Set(controlMode, speed.FL * inv.FL * SCALE_FACTOR);
		frontRightDrive->Set(controlMode, speed.FR * inv.FR * SCALE_FACTOR);
		rearLeftDrive->  Set(controlMode, speed.RL * inv.RL * SCALE_FACTOR);
		rearRightDrive-> Set(controlMode, speed.RR * inv.RR * SCALE_FACTOR);
	}
	else {
		frontLeftDrive-> Set(controlMode, speed.RR * inv.FL * SCALE_FACTOR);
		frontRightDrive->Set(controlMode, speed.RL * inv.FR * SCALE_FACTOR);
		rearLeftDrive->  Set(controlMode, speed.FR * inv.RL * SCALE_FACTOR);
		rearRightDrive-> Set(controlMode, speed.FL * inv.RR * SCALE_FACTOR);
	}
}

void DriveBase::SetConstantVelocity(double twistInput, double velocity) {
	Crab(twistInput, velocity, 0, true);
}

void DriveBase::SetTargetAngle(double angle) {
	driveControlTwist->SetSetpoint(angle);
}

double DriveBase::GetTwistControlOutput() {
	return driveControlTwist->Get();
}

double DriveBase::GetTwistControlError() {
	return driveControlTwist->GetError();
}

void DriveBase::SetTargetDriveDistance(double distance, double maxSpeed) {
	driveControlSpeedController->SetSetpoint(distance);
	driveControlSpeedController->SetOutputRange(-maxSpeed, maxSpeed);
	driveControlEncoderSource->SetInitialEncoderValue();
}

double DriveBase::GetDriveControlEncoderPosition() {
	return driveControlEncoderSource->PIDGet();
}

double DriveBase::GetDriveControlOutput() {
	return driveControlDistanceSpeed->Get();
}

double DriveBase::GetDriveControlError() {
	return driveControlSpeedController->GetError();
}

double DriveBase::GetDriveControlP() {
	return driveControlSpeedController->GetP();
}

Wheelbase DriveBase::GetWheelbase() {
	return wheelbase;
}

double DriveBase::GetCrabTwistOutput() {
	return crabSpeedTwist->Get();
}

void DriveBase::Instrument() {
	SmartDashboard::PutNumber("FL V", frontLeftDrive->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("FR V", frontRightDrive->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("RL V", rearLeftDrive->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("RR V", rearRightDrive->GetSelectedSensorVelocity(0));

	SmartDashboard::PutNumber("FL SErr", frontLeftSteer->GetClosedLoopError(0));
	SmartDashboard::PutNumber("FR SErr", frontRightSteer->GetClosedLoopError(0));
	SmartDashboard::PutNumber("RL SErr", rearLeftSteer->GetClosedLoopError(0));
	SmartDashboard::PutNumber("RR SErr", rearRightSteer->GetClosedLoopError(0));

	SmartDashboard::PutNumber("FL Pos", frontLeftDrive->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("FR Pos", frontRightDrive->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("RL Pos", rearLeftDrive->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("RR Pos", rearRightDrive->GetSelectedSensorPosition(0));
}


