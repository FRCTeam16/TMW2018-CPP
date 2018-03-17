/*
 * Elevator.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_ELEVATOR_H_
#define SRC_SUBSYSTEMS_ELEVATOR_H_
#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "../RobotMap.h"
#include "SubsystemManager.h"
#include "ElevatorMagicMotionManager.h"



class Elevator : SubsystemManager, public frc::Subsystem {
public:
	Elevator();
	virtual ~Elevator();

	enum ElevatorPosition {
		kFloor, kSwitch, kLowScale, kClimb, kDoClimb, kHighScale
	};
	const int ELEVATOR_POSITION_COUNT = 6;

	enum RunMode {
		kManual, kMagic
	};


	void InitDefaultCommand() {}
	void Periodic() {}

	void Init() override;
	void Run() override;
	void Instrument() override;

	void SetOpenLoopPercent(double openLoopPercent);

	void SetInitialPosition();
	ElevatorPosition GetElevatorPosition();
	void SetElevatorPosition(ElevatorPosition position);
	void SetElevatorSetpoint(int setpoint);
	bool InPosition();
	void IncreaseElevatorPosition();
	void DecreaseElevatorPosition();
	void HoldPosition();
	bool IsAboveSwitch();

	void HoldClimb();
	void UnholdClimb();

	void ToggleShifter();
	void ShiftLow();
	void ShiftHigh();

	void SetHomePosition();

	int GetElevatorEncoderPosition();
	std::tuple<double, double> GetElevatorMotorCurrents();


private:
	std::shared_ptr<WPI_TalonSRX> mainElevatorMotor = RobotMap::elevatorElevatorMotor1;
	std::shared_ptr<WPI_TalonSRX> followerElevatorMotor = RobotMap::elevatorElevatorMotor2;
	const std::vector<std::shared_ptr<WPI_TalonSRX>> motors { mainElevatorMotor, followerElevatorMotor };
	std::shared_ptr<DoubleSolenoid> shifter = RobotMap::mastShifter;
	ElevatorPosition elevatorPosition = ElevatorPosition::kFloor;
	double openLoopPercent = 0.0;
	int elevatorPositionThreshold;
	double setpoint = 0.0;
	std::unique_ptr<ElevatorMagicMotionManager> magicMotionManager;
	RunMode runMode = RunMode::kManual;
	bool shifterState = false;       // false = reverse = low, true = forward = high

	void SetShift(bool state);
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
