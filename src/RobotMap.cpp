// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.



#include "RobotMap.h"

std::shared_ptr<WPI_TalonSRX> RobotMap::mastleftDart;
std::shared_ptr<WPI_TalonSRX> RobotMap::mastrightDart;
std::shared_ptr<DoubleSolenoid> RobotMap::mastShifter;
std::shared_ptr<Compressor> RobotMap::compressor;
std::shared_ptr<WPI_VictorSPX> RobotMap::intakeLeftIntakeMotor;
std::shared_ptr<WPI_VictorSPX> RobotMap::intakeRightIntakeMotor;
std::shared_ptr<Solenoid> RobotMap::intakeExtendActuator;
std::shared_ptr<WPI_TalonSRX> RobotMap::elevatorElevatorMotor1;
std::shared_ptr<WPI_TalonSRX> RobotMap::elevatorElevatorMotor2;
std::shared_ptr<BSGyro> RobotMap::gyro;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontLeftDrive;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontRightDrive;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearLeftDrive;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearRightDrive;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontLeftSteer;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontRightSteer;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearLeftSteer;
std::shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearRightSteer;
std::shared_ptr<PowerDistributionPanel> RobotMap::powerDistributionPanel;

//std::shared_ptr<WPI_TalonSRX> RobotMap::intakeRotateMotor;
//std::shared_ptr<DigitalInput> RobotMap::intakeRotateEncoder;


void RobotMap::init() {

    driveBaseFrontLeftDrive.reset(new WPI_TalonSRX(1));
    driveBaseFrontRightDrive.reset(new WPI_TalonSRX(3));
    driveBaseRearLeftDrive.reset(new WPI_TalonSRX(5));
    driveBaseRearRightDrive.reset(new WPI_TalonSRX(7));

    driveBaseFrontLeftSteer.reset(new WPI_TalonSRX(2));
    driveBaseFrontRightSteer.reset(new WPI_TalonSRX(4));
    driveBaseRearLeftSteer.reset(new WPI_TalonSRX(6));
    driveBaseRearRightSteer.reset(new WPI_TalonSRX(8));

    powerDistributionPanel.reset(new PowerDistributionPanel(0));

    compressor.reset(new Compressor(0));
    mastShifter.reset(new DoubleSolenoid(2, 3));

    mastleftDart.reset(new WPI_TalonSRX(9));
    mastrightDart.reset(new WPI_TalonSRX(10));

    intakeLeftIntakeMotor.reset(new WPI_VictorSPX(11));
    intakeRightIntakeMotor.reset(new WPI_VictorSPX(12));
    intakeExtendActuator.reset(new Solenoid(0));

//    intakeRotateMotor.reset(new WPI_TalonSRX(15));
//    intakeRotateEncoder.reset(new DigitalInput(0));


    elevatorElevatorMotor1.reset(new WPI_TalonSRX(13));
    elevatorElevatorMotor2.reset(new WPI_TalonSRX(14));

    gyro.reset(new BSGyro(elevatorElevatorMotor2.get()));
}
