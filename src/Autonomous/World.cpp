/*
 * World.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */
#include "WPILib.h"
#include <Autonomous/World.h>


World::World() {
//	timer.reset(new frc::Timer());
}

World::~World() {
}

void World::Init() {
//	timer->Start();
}

double World::GetClock() const {
//	return timer->Get();
	return frc::Timer::GetFPGATimestamp();
}

