
#ifndef SRC_SUBSYSTEMS_MAST_H_
#define SRC_SUBSYSTEMS_MAST_H_

#include <iostream>
#include <map>

#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "../RobotMap.h"
#include "SubsystemManager.h"
#include "DartConstraint.h"
#include "DartMagicMotionManager.h"


class Mast : SubsystemManager, public frc::Subsystem {
public:
	Mast();
	virtual ~Mast();

	/** Order matters - ManualPosition(0) is ignored for inc/dec operations */
	enum MastPosition {
		kManualPosition, kDrive, kVertical, kClimb, kCurl, kStartingPosition
	};

	enum RunMode {
		kManual, kPositionControl, kMagic
	};

	void Init() override;
	void Run() override;
	void Instrument() override;

	void SetMastPosition(Mast::MastPosition mastPosition);
	bool InPosition();

	void BeginCurl();
	void HoldCurl();
	void UndoHoldCurl();

	void SetTarget(int target);
	void AdjustTarget(int ticks);

	void ZeroLeftMotorSpeedIfManual();

private:
   std::shared_ptr<WPI_TalonSRX> leftDart = RobotMap::mastleftDart;
   std::shared_ptr<WPI_TalonSRX> rightDart = RobotMap::mastrightDart;
   const std::vector<std::shared_ptr<WPI_TalonSRX>> darts = {leftDart, rightDart};
   std::shared_ptr<DartMagicMotionManager> magicMotionManager;	// should probably be unique_ptr

   std::shared_ptr<DartConstraint> leftConstraint;
   std::shared_ptr<DartConstraint> rightConstraint;

   double leftMotorSpeed;
   double rightMotorSpeed;
   int magicTarget;

   Mast::RunMode runMode = Mast::RunMode::kManual;
   Mast::MastPosition mastPosition = Mast::MastPosition::kManualPosition;
   int DART_DIVERGENCE_THRESHOLD = 30;
   std::map<Mast::MastPosition, int> positionLookup;
   int dartPositionThreshold;

   bool AreDartsDiverged();
   void SetLeftMotorSpeed(double speed);
   void SetRightMotorSpeed(double speed);




};



#endif /* SRC_SUBSYSTEMS_MAST_H_ */
