/*
 * AveragingDriveEncoderPIDSource.h
 *
 *  Created on: Mar 26, 2017
 *      Author: User
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_FRONTTWOAVERAGINGDRIVEENCODERPIDSOURCE_H_
#define SRC_SUBSYSTEMS_DRIVE_FRONTTWOAVERAGINGDRIVEENCODERPIDSOURCE_H_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "../../Util/DriveInfo.h"

class FrontTwoAveragingDriveEncoderPIDSource : public PIDSource {
public:
	FrontTwoAveragingDriveEncoderPIDSource(DriveInfo<std::shared_ptr<WPI_TalonSRX>> _motor);
	virtual ~FrontTwoAveragingDriveEncoderPIDSource();
	virtual double PIDGet();
	void SetInitialEncoderValue();
	void SetShowDebug(bool _showDebug);
private:
	DriveInfo<std::shared_ptr<WPI_TalonSRX>> motor;
	DriveInfo<double> initialEncoderValue;
	double CalculateAverage(const DriveInfo<double> &error, const DriveInfo<bool> &motorEnabled);
	bool showDebug = false;
};

#endif /* SRC_SUBSYSTEMS_DRIVE_FRONTTWOAVERAGINGDRIVEENCODERPIDSOURCE_H_ */
