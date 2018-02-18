/*
 * DartMagicMotionManager.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_DARTMAGICMOTIONMANAGER_H_
#define SRC_SUBSYSTEMS_DARTMAGICMOTIONMANAGER_H_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <Util/PrefUtil.h>
#include "DartConstraint.h"


class DartMagicMotionManager {

private:
    const static int SET_TIMEOUT = 0;
    std::shared_ptr<TalonSRX> talon;
    std::shared_ptr<DartConstraint> talonConstraint;
    std::shared_ptr<TalonSRX> follower;
    std::shared_ptr<DartConstraint> followerConstraint;
    Notifier *rightDartControlNotifier;

    static std::shared_ptr<DartMagicMotionManager> INSTANCE;



    bool running = false;
    int lastErrorSign = 0;

    DartMagicMotionManager(
       		std::shared_ptr<TalonSRX> _talon,
   			std::shared_ptr<DartConstraint> _talonConstraint,
   			std::shared_ptr<TalonSRX> _follower,
   			std::shared_ptr<DartConstraint> _followerConstraint) :
       			talon(_talon), talonConstraint(_talonConstraint),
   				follower(_follower), followerConstraint(_followerConstraint)
   	{
   		rightDartControlNotifier = new Notifier(DartMagicMotionManager::OnNotify);
   		rightDartControlNotifier->StartPeriodic(0.005);   // 5 milliseconds
   		talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 5, 0);
   		follower->SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 5, 0);
   		follower->SetControlFramePeriod(ControlFrame::Control_3_General, 5);
   		std::cout << "Constructed new DartMagicMotionManager\n";
   	}

    void SetProcessFollowerRunning(bool _running) {
		running = _running;
	}

public:
    const int SLOT = 0;

    static std::shared_ptr<DartMagicMotionManager> GetInstance(
    		std::shared_ptr<TalonSRX> _talon,
			std::shared_ptr<DartConstraint> _talonConstraint,
			std::shared_ptr<TalonSRX> _follower,
			std::shared_ptr<DartConstraint> _followerConstraint) {

    	if (INSTANCE == 0) {
    		INSTANCE = std::shared_ptr<DartMagicMotionManager>(
    				new DartMagicMotionManager(_talon, _talonConstraint, _follower, _followerConstraint));
    	}

    	return INSTANCE;
    }


    void InitDefaultPID() {
        int V = PrefUtil::getSetInt("DartMagicV", 50);
        int A = PrefUtil::getSetInt("DartMagicA", 50);
        double P = PrefUtil::getSet("DartMagicP", 0);
        double F = PrefUtil::getSet("DartMagicF", 17);
        double I = PrefUtil::getSet("DartMagicI", 0.001);

        talon->SelectProfileSlot(SLOT, 0);
        int error = talon->ConfigMotionCruiseVelocity(V, SET_TIMEOUT);
        std::cout << "Error setting velocity? " << error << "\n";
        talon->ConfigMotionAcceleration(A, SET_TIMEOUT);
        talon->Config_kP(SLOT, P, 0);
        talon->Config_kI(SLOT, I, 0);
        talon->Config_kD(SLOT, 0, 0);
        talon->Config_kF(SLOT, F, 0);

        double fP = PrefUtil::getSet("DartFollowP", 15);
        double fI = PrefUtil::getSet("DartFollowI", 0.12);
        double fD = PrefUtil::getSet("DartFollowD", 0.01);

        follower->Config_kP(SLOT, fP, 0);
        follower->Config_kI(SLOT, fI, 0);
        follower->Config_kD(SLOT, fD, 0);
        follower->Config_kF(SLOT, 0, 0);
        std::cout << "Configured Default Dart PID\n";
    }

    void InitCurlPID() {
        int V = PrefUtil::getSet("DartMagicCurlV", 50);
        int A = PrefUtil::getSet("DartMagicCurlA", 50);

        double P = PrefUtil::getSet("DartMagicCurlP", 100);
        double F = PrefUtil::getSet("DartMagicCurlF", 17);
        double I = PrefUtil::getSet("DartMagicCurlI", 0.001);

        talon->SelectProfileSlot(SLOT, 0);
        int error = talon->ConfigMotionCruiseVelocity(V, SET_TIMEOUT);
        std::cout << "Error setting velocity? " << error << "\n";
        talon->ConfigMotionAcceleration(A, SET_TIMEOUT);
        talon->Config_kP(SLOT, P, SET_TIMEOUT);
        talon->Config_kI(SLOT, I, SET_TIMEOUT);
        talon->Config_kD(SLOT, 0, SET_TIMEOUT);
        talon->Config_kF(SLOT, F, SET_TIMEOUT);

        double fP = PrefUtil::getSet("DartFollowCurlP", 100);
        double fI = PrefUtil::getSet("DartFollowCurlI", 0.12);
        double fD = PrefUtil::getSet("DartFollowCurlD", 0.01);

        follower->Config_kP(SLOT, fP, SET_TIMEOUT);
        follower->Config_kI(SLOT, fI, SET_TIMEOUT);
        follower->Config_kD(SLOT, fD, SET_TIMEOUT);
        follower->Config_kF(SLOT, 0, SET_TIMEOUT);
        std::cout << "Configured Curl PID\n";
    }

    void Run(int target) {
        talon->Set(ControlMode::MotionMagic, target);
        SetProcessFollowerRunning(true);
        // TODO: Might be nice to test getting the calculated target from profile vs. instantaneous value for smoother follower tracking
        //        talon.getActiveTrajectoryPosition(); check docs, but period may be 160ms for active trj p
    }

    void Stop() {
    	SetProcessFollowerRunning(false);
    }


    void ProcessFollower() {
    	if (running) {
		   int currentPosition = talon->SetSelectedSensorPosition(0, 0, 0);
		   int followerTarget = DartConstraint::mapDartConstraint(currentPosition, talonConstraint, followerConstraint);

		   int closedLoopError = follower->GetClosedLoopError(0);
		   int currentErrorSign = copysign(1, closedLoopError);
		   if (currentErrorSign != lastErrorSign) {
			   /*logger.info(String.format("Closed Loop Error: %d Clearing accumulator %d -> %d",
					   closedLoopError, lastErrorSign, currentErrorSign));*/
			   follower->SetIntegralAccumulator(0.0, 0, 0);
		   }
		   lastErrorSign = currentErrorSign;

		   //  logger.info("======> TARGET = " + target + " | CURRENT -> " + currentPosition + " | FOLLOWER TARGET -> " + followerTarget);
		   follower->Set(ControlMode::Position, followerTarget);
	   }
    }

    static void OnNotify() {
    	if (INSTANCE != nullptr) {
    		INSTANCE->ProcessFollower();
    	}
    }



}; // end dart magic motion manager




#endif /* SRC_SUBSYSTEMS_DARTMAGICMOTIONMANAGER_H_ */
