/*
 * ConcurrentStep.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_AUTONOMOUS_STEPS_CONCURRENTSTEP_H_
#define SRC_AUTONOMOUS_STEPS_CONCURRENTSTEP_H_

#include <vector>
#include <Autonomous/Step.h>

class ConcurrentStep: public Step {
public:
	ConcurrentStep(std::initializer_list<Step*> stepList);
	virtual ~ConcurrentStep();
	bool Run(std::shared_ptr<World> world) override;
	const CrabInfo GetCrabInfo();
private:
	Step* driveStep;
	std::vector<Step*> steps;
};

class WrappedStep {
public:
	WrappedStep(Step* _step) : step(_step) {}
	bool Run(std::shared_ptr<World> world);
private:
	Step* step;
	bool finished;
};

#endif /* SRC_AUTONOMOUS_STEPS_CONCURRENTSTEP_H_ */
