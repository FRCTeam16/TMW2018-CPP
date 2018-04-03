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

	double driveDistance = 0;
	bool teamSideMode = false;
	bool autoTraverse = false;
public:
	World();
	virtual ~World() {}

	void Init();							// perform world initialization
	double GetClock() const;				// time elapsed since Init() in seconds
	bool IsRed();
	FieldInfo GetFieldInfo();
	void SetFieldInfo(FieldInfo fieldInfo);
	AutoStartPosition GetStartPosition() { return startPosition; }
	void SetStartPosition(AutoStartPosition pos) { startPosition = pos; }

	void SetDriveDistance(double d) { driveDistance = d; }
	double GetDriveDistance() { return driveDistance; }

	void SetTeamSideMode(bool _mode) { teamSideMode = _mode; }
	bool GetTeamSideMode() { return teamSideMode; }

	void SetAutoTraverse(bool _traverse) { autoTraverse = _traverse; }
	bool GetAutoTraverse() { return autoTraverse; }
};

#endif /* SRC_AUTONOMOUS_WORLD_H_ */
