/*
 * AutoManager.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#include <Autonomous/AutoManager.h>
#include "../RobotMap.h"
#include "Strategies/DebugAutoStrategy.h"
#include <Autonomous/Strategies/DebugAutoStrategy.h>
#include <Robot.h>



AutoManager::AutoManager() :
		strategies(new frc::SendableChooser<void*>())
{
	strategies->AddDefault("999 - Debug Auto Strategy", (void *) AutoStrategy::kDebug);
	frc::SmartDashboard::PutData("Autonomous Strategy", strategies.get());

	const AutoStrategy selectedKey = static_cast<AutoStrategy>((int) strategies->GetSelected());
	frc::SmartDashboard::PutNumber("Selected Auto", selectedKey);
}

AutoManager::~AutoManager() {
}

std::unique_ptr<Strategy> AutoManager::CreateStrategy(const AutoStrategy &key) {
	const frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	const bool isRed =  alliance == frc::DriverStation::Alliance::kRed;
	//|| alliance == frc::DriverStation::Alliance::kInvalid;

	std::cout << "AutoManager::CreateStrategy -> isRed = " << isRed << "\n";

	Strategy *strategy = 0;
	switch (key) {
	case kDebug:
		strategy = new DebugAutoStrategy();
		break;
	default:
		// TODO: Fill in sane default
		std::cerr << "No valid strategy selected";
	}
	return std::unique_ptr<Strategy>(strategy);
}


void AutoManager::Init(std::shared_ptr<World> world) {
	std::cout << "AutoMan Init\n";
	const AutoStrategy selectedKey = static_cast<AutoStrategy>((int) strategies->GetSelected());
	frc::SmartDashboard::PutNumber("Selected Auto", selectedKey);
	std::cout << "Selected Strategy: " << selectedKey << "\n";
	currentStrategy = CreateStrategy(selectedKey);
	if (!currentStrategy) {
		std::cerr << "NO AUTONOMOUS STRATEGY FOUND\n";
	}
	RobotMap::gyro->ZeroYaw();

	startTime = -1;
	finalPhaseFired = false;

	std::cout << "AutoManager::Init COMPLETE\n";
}

void AutoManager::Periodic(std::shared_ptr<World> world) {
//	std::cout << "AutoMan Periodic\n";
	const double currentTime = world->GetClock();
	if (currentStrategy) {

        // Perform global startup runtime system actions here
		if (startTime < 0) {
			startTime = currentTime;
		}

        // Perform any global post-init runtime system actions here
		if ((currentTime - startTime) > 1) {
		}

        // Perform any global final system actions here
		if (((currentTime - startTime) > 13) && !finalPhaseFired) {
			finalPhaseFired = true;

		}
		currentStrategy->Run(world);
	}
}

void AutoManager::Instrument() {
	const AutoStrategy selectedKey = static_cast<AutoStrategy>((int) strategies->GetSelected());
	frc::SmartDashboard::PutNumber("Selected Auto", selectedKey);
}
