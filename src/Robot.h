// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef _ROBOT_H
#define _ROBOT_H

#include <vector>

#include "WPILib.h"
#include "Commands/Command.h"
#include "RobotMap.h"
#include "LiveWindow/LiveWindow.h"


#include "Subsystems/DriveBase.h"
#include "Subsystems/Ramp.h"
#include "Subsystems/Intake.h"
#include "Subsystems/Elevator.h"
#include "Subsystems/Mast.h"
#include "Climb/ClimbProcess.h"

#include "Subsystems/SubsystemManager.h"



#include "OI.h"

class Robot : public frc::TimedRobot {
public:
	frc::Command* autonomousCommand = nullptr;
	static std::unique_ptr<OI> oi;
	frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
	frc::SendableChooser<frc::Command*> chooser;

	static std::shared_ptr<Ramp> ramp;
	static std::shared_ptr<DriveBase> driveBase;
	static std::shared_ptr<Intake> intake;
	static std::shared_ptr<Elevator> elevator;
	static std::shared_ptr<Mast> mast;


	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;

	void InitSubsystems();
	void RunSubsystems();
	void InstrumentSubsystems();
private:
	std::unique_ptr<ClimbProcess> climbProcess;

};
#endif
