/*
 * CollisionDetector.cpp
 */

#include "CollisionDetector.h"
#include <iostream>

CollisionDetector::CollisionDetector(std::shared_ptr<BSGyro> imu_, double threshold_ ) :
	imu(imu_), threshold(threshold_) {
}

CollisionDetector::~CollisionDetector() {
}

bool CollisionDetector::Detect() {
	short xyz[3];
	imu->GetPigeon()->GetBiasedAccelerometer(xyz);

	const double UNIT_SCALE = 16384.0; //  = 1G  http://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html#a211525ea83d9728416661238a2a5402a

	double current_accel_x = xyz[0] / UNIT_SCALE;;
	double current_jerk_x = current_accel_x - last_accel_x;
	last_accel_x = current_accel_x;

	double current_accel_y = xyz[1] / UNIT_SCALE;
	double current_jerk_y = current_accel_y - last_accel_y;
	last_accel_y = current_accel_y;

	frc::SmartDashboard::PutNumber("Current Jerk X", current_jerk_x);
	frc::SmartDashboard::PutNumber("Current Jerk Y", current_jerk_y);

	std::cout <<  "Jerk: X " << current_jerk_x << " | Y " << current_jerk_y << "  : " << threshold << "\n";


	return ((fabs(current_jerk_x) > threshold) ||
			(fabs(current_jerk_y) > threshold));
}


