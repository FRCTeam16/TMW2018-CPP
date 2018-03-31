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

	void Start(double speed);
	void Start();
	void Stop();
	void Eject();
	void Eject(double speed);

	bool IsPickupTriggered();
	void ResetPickupTriggerState();
	void SetExtendSolenoidState(bool extend);
	void ToggleExtendSolenoidState();
	void SetPickupTriggered(bool triggered);

	void RotateIntakeUp();
	void RotateIntakeDown();
	void SetRotateIntakePosition(int position, bool _direction);

	void SetRotateIntakeSpeed(double _speed);

private:

	struct RotationState {
		int targetPosition = 0;
		bool direction = false;		// false = down, true = up
		bool rotating = false;

		void setPosition(int position, bool _direction) {
			rotating = true;
			targetPosition = position;
			direction = _direction;
		}

		double getMotorOutput(const std::shared_ptr<Counter> counter) {
			if (rotating) {
				if (counter->Get() < targetPosition) {
					return (direction) ? -1.0 : 1.0;
				} else {
					rotating = false;
					counter->Reset();
					return 0.0;
				}
			}
			return 0.0;
		}
	};

	 std::shared_ptr<WPI_VictorSPX> leftIntakeMotor = RobotMap::intakeLeftIntakeMotor;
	 std::shared_ptr<WPI_VictorSPX> rightIntakeMotor = RobotMap::intakeRightIntakeMotor;
	 std::shared_ptr<Solenoid> extendSolenoid = RobotMap::intakeExtendActuator;
	 std::shared_ptr<WPI_TalonSRX> rotateMotor = RobotMap::intakeRotateMotor;
	 std::shared_ptr<Counter> rotateCounter;

	 IntakeState state = IntakeState::kStop;
	 bool extendSolenoidState = true;
	 double intakeSpeed = 1.0;
	 double switchEjectSpeed = 1.0;
	 double scaleEjectSpeed = 0.75;
	 bool pickupTriggered = false;
	 double pickupStartTime = 0.0;
	 const double pickupTimeout = 0.5;
	 std::shared_ptr<ThresholdCounter> leftIntakeAmpThresholdCounter;
	 std::shared_ptr<ThresholdCounter> rightIntakeAmpThresholdCounter;
	 double targetEjectSpeed = 0.0;

	 double rotateSpeed = 0.0;
	 RotationState rotationState;
	 int minRotatePosition = 0;
	 int maxRotatePosition = 5000;

	 long loopCounter = 0;		// used for instrumentation


};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
