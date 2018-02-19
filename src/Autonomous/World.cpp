/*
 * World.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */
#include "WPILib.h"
#include <Autonomous/World.h>
#include "DriverStation.h"


World::World() {
	DriverStation::Alliance alliance = DriverStation::GetInstance().GetAlliance();
	isRed = DriverStation::Alliance::kRed == alliance;
	fieldInfo = FieldInfo();
}


void World::Init() {
//	timer->Start();
}

double World::GetClock() const {
//	return timer->Get();
	return frc::Timer::GetFPGATimestamp();
}

FieldInfo World::GetFieldInfo() {
	return fieldInfo;
}

bool World::IsRed() {
	return isRed;
}
