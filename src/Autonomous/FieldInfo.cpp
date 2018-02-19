/*
 * FieldInfo.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#include <Autonomous/FieldInfo.h>
#include "DriverStation.h"
#include <iostream>
#include <string>

FieldInfo::FieldInfo() {
	std::string gameData = DriverStation::GetInstance().GetGameSpecificMessage();
	if(gameData.length() == 3)
	{
		switchLocation = gameData[0] == 'L' ? Location::Left : Location::Right;
		scaleLocation = gameData[1] == 'L' ? Location::Left : Location::Right;
		farSwitchLocation = gameData[2] == 'L' ? Location::Left : Location::Right;
	} else {
		std::cerr << "Did not receive correct game specific message to parse [" << gameData << "]";
		switchLocation = scaleLocation = farSwitchLocation = Location::Unknown;
	}

}

FieldInfo::~FieldInfo() {
	// TODO Auto-generated destructor stub
}

