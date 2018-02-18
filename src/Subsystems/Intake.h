/*
 * Intake.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_
#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "../RobotMap.h"
#include "SubsystemManager.h"
#include "../Util/ThresholdCounter.h"

enum IntakeState {
		kStop, kIntake, kEject
	};

class Intake : SubsystemManager, public frc::Subsystem {
public:
	Intake();
	virtual ~Intake();
	void InitDefaultCommand() {}
	void Periodic() {}

	void Init() override;
	void Run() override;
	void Instrument() override;

	void SetMotorPercent(double amount);
	double GetLeftIntakeCurrent();
	double GetRightIntakeCurrent();

	void Start();
	void Stop();
	void Eject();
	void Eject(double speed);

	bool IsPickupTriggered();
	void ResetPickupTriggerState();
	void SetExtendSolenoidState(bool extend);
	void ToggleExtendSolenoidState();
	void SetPickupTriggered(bool triggered);
	void SetRotateSolenoidState(bool rotate);
	void ToggleRotateSolenoidState();


private:
	 std::shared_ptr<WPI_VictorSPX> leftIntakeMotor = RobotMap::intakeLeftIntakeMotor;
	 std::shared_ptr<WPI_VictorSPX> rightIntakeMotor = RobotMap::intakeRightIntakeMotor;
	 std::shared_ptr<Solenoid> extendSolenoid = RobotMap::intakeExtendActuator;
	 std::shared_ptr<Solenoid> rotateSolenoid = RobotMap::intakeRotateActuator;

	 IntakeState state = IntakeState::kStop;
	 bool extendSolenoidState = false;
	 bool rotateSolenoidState = false;
	 double intakeSpeed = 1.0;
	 double defaultEjectSpeed = -1.0;
	 bool pickupTriggered = false;
	 std::unique_ptr<ThresholdCounter> leftIntakeAmpThresholdCounter;
	 std::unique_ptr<ThresholdCounter> rightIntakeAmpThresholdCounter;
	 double targetEjectSpeed = 0.0;

};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
