#ifndef SRC_AUTONOMOUS_STRATEGIES_CHISIDESTRATEGY_H_
#define SRC_AUTONOMOUS_STRATEGIES_CHISIDESTRATEGY_H_

#include <WPILib.h>
#include <Autonomous/Strategy.h>
#include <Autonomous/World.h>
#include <Subsystems/Mast.h>

class ChiSideStrategy: public StepStrategy {
public:
	ChiSideStrategy(std::shared_ptr<World> world);
	virtual ~ChiSideStrategy() {}

private:
	bool isLeft = false;
	bool isRight = false;
	int inv = 1;			// invert angle multiplier (should be for left starts)
	double startAngle;		// initial orientation of robot

	void StartInitialPose(Mast::MastPosition position = Mast::MastPosition::kVertical);	// run before other code paths to begin configuring robot pose
	void DoSwitchScale();		// runs a switch score - pickup - scale score

	void DoFirstScale();		// first cube scale
	void DoSecondScale();		// second cube scale after intitial score

	void DoSwitchPickup();		// runs a switch score - pickup
	void DoTraverse();			// runs a traverse

	void CrossLine();

	// Helpers

	void DoSecondCubePickup(double robotAngle, double xDriveDistance);
	void DoThirdCubePickup(double robotAngle, double xDriveDistance);

	void DoSecondCubeScale();

};

#endif /* SRC_AUTONOMOUS_STRATEGIES_CHISIDESTRATEGY_H_ */
