/*
 * ElevatorMagicMotionManager.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_ELEVATORMAGICMOTIONMANAGER_H_
#define SRC_SUBSYSTEMS_ELEVATORMAGICMOTIONMANAGER_H_

#include "ctre/Phoenix.h"
#include <Util/PrefUtil.h>

class ElevatorMagicMotionManager {
public:
	ElevatorMagicMotionManager(std::shared_ptr<TalonSRX> _talon) : talon(_talon) {
	}

    void Init() {
        double P = PrefUtil::getSet("ElevatorHighSpeedP", 1);
        double F = PrefUtil::getSet("ElevatorHighSpeedF", 0.18);
        int V = PrefUtil::getSetInt("ElevatorHighSpeedV", 5592);
        int A = PrefUtil::getSetInt("ElevatorHighSpeedA", 5592);

        talon->ConfigMotionCruiseVelocity(V, 0);
        talon->ConfigMotionAcceleration(A, 0);
        talon->Config_kP(0, P, 0);
        talon->Config_kI(0, 0, 0);
        talon->Config_kD(0, 0, 0);
        talon->Config_kF(0, F, 0);
        talon->ClearMotionProfileTrajectories();
        talon->ClearMotionProfileHasUnderrun(0);
    }

    void Run(double target) {
        talon->Set(ControlMode::MotionMagic, target);
    }

private:
    const std::shared_ptr<TalonSRX> talon;
};



#endif /* SRC_SUBSYSTEMS_ELEVATORMAGICMOTIONMANAGER_H_ */
