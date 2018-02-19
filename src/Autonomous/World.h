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
#include "AutoPositions.h"

class AutoManager;

class World {
private:
	std::unique_ptr<Timer> timer;			// tracks time in autonomous world
	bool isRed;
	FieldInfo fieldInfo;
	AutoStartPosition startPosition;
public:
	World();
	virtual ~World() {}

	void Init();							// perform world initialization
	double GetClock() const;				// time elapsed since Init() in seconds
	bool IsRed();
	FieldInfo GetFieldInfo();
	AutoStartPosition GetStartPosition() { return startPosition; }
	void SetStartPosition(AutoStartPosition pos) { startPosition = pos; }
};

#endif /* SRC_AUTONOMOUS_WORLD_H_ */
