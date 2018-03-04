/*
 * AutoManager.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#include <Autonomous/AutoManager.h>
#include "AutoPositions.h"
#include "../RobotMap.h"
#include "Strategies/DebugAutoStrategy.h"
#include <Autonomous/Strategies/DebugAutoStrategy.h>
#include <Autonomous/Strategies/CenterSwitchStrategy.h>
#include <Autonomous/Strategies/SideStrategy.h>
#include <Robot.h>



AutoManager::AutoManager() :
		positions(new frc::SendableChooser<int>()),
		strategies(new frc::SendableChooser<int>())
{
	strategies->AddObject("1 - Center Switch", AutoStrategy::kCenterSwitch);
	strategies->AddDefault("2 - Side Start", AutoStrategy::kSide);
	strategies->AddObject("3 - Debug Auto Strategy", AutoStrategy::kDebug);
	std::cout << "AutoManager::Added Strategies\n";

	std::cout << "AutoManager::AutoManager() start\n";
	positions->AddDefault("2 - Center", AutoStartPosition::kCenter);
	positions->AddObject("1 - Left",  AutoStartPosition::kLeft);
	positions->AddObject("3 - Right", AutoStartPosition::kRight);
//	std::cout << "AutoManager::Added Positions\n";


	frc::SmartDashboard::PutData("Autonomous Start Pos", positions.get());
	frc::SmartDashboard::PutData("Autonomous Strategy7", strategies.get());
	std::cout << "AutoManager::AutoManager() finished\n";
}

AutoManager::~AutoManager() {
}

std::unique_ptr<Strategy> AutoManager::CreateStrategy(const AutoStrategy &key, std::shared_ptr<World> world) {
	const frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	const bool isRed =  alliance == frc::DriverStation::Alliance::kRed;

	std::cout << "AutoManager::CreateStrategy -> isRed = " << isRed << "\n";

	Strategy *strategy = 0;
	switch (key) {
	case kDebug:
		std::cout << "AUTOMAN: Selected DEBUG \n";
		strategy = new DebugAutoStrategy(world);
		break;
	case kCenterSwitch:
		std::cout << "AUTOMAN: Selected Center Switch\n";
		strategy = new CenterSwitchStrategy(world);
		break;
	case kSide:
		std::cout << "AUTOMAN: Selected Side Strategy\n";
		strategy = new SideStrategy(world);
		break;
	default:
		// TODO: Fill in sane default
		std::cerr << "No valid strategy selected\n";
	}
	return std::unique_ptr<Strategy>(strategy);
}


void AutoManager::Init(std::shared_ptr<World> world) {
	std::cout << "AutoMan Init\n";

	const AutoStartPosition selectedPosition = static_cast<AutoStartPosition>(positions->GetSelected());
	std::cout << "AutoMan Position selectedKey: " << selectedPosition << "\n";
	world->SetStartPosition(selectedPosition);

	const AutoStrategy selectedKey = static_cast<AutoStrategy>(2);
	std::cout << "AutoMan Init selectedKey: " << selectedKey << "\n";

	currentStrategy = CreateStrategy(selectedKey, world);
	if (!currentStrategy) {
		std::cerr << "NO AUTONOMOUS STRATEGY FOUND\n";
	}

	RobotMap::gyro->ZeroYaw();
	currentStrategy->Init(world);

	startTime = -1;
	finalPhaseFired = false;
	std::cout << "AutoManager::Init COMPLETE\n";
}

void AutoManager::Periodic(std::shared_ptr<World> world) {
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
	const AutoStrategy selectedKey = static_cast<AutoStrategy>((int) 1);
	frc::SmartDashboard::PutNumber("Selected Auto", selectedKey);
	const AutoStartPosition selectedPosition = static_cast<AutoStartPosition>(positions->GetSelected());
	frc::SmartDashboard::PutNumber("Auto Selected Position", selectedPosition);
}
