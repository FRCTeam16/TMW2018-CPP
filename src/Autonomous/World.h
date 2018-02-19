/*
 * World.h
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_WORLD_H_
#define SRC_AUTONOMOUS_WORLD_H_

#include "WPILib.h"
#include "FieldInfo.h"

class World {
public:
	World();
	virtual ~World() {}
	void Init();							// perform world initializatoin
	double GetClock() const;				// time elapsed since Init() in seconds
	bool IsRed();
	FieldInfo GetFieldInfo();
private:
	std::unique_ptr<Timer> timer;			// tracks time in autonomous world
	bool isRed;
	FieldInfo fieldInfo;
};

#endif /* SRC_AUTONOMOUS_WORLD_H_ */
