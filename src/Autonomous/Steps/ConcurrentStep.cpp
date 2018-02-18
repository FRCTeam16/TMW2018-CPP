
#include <iostream>
#include <Autonomous/Steps/ConcurrentStep.h>

ConcurrentStep::ConcurrentStep(std::initializer_list<Step*> stepList) : steps(stepList) {
	std::cout << "Loaded " << steps.size() << " steps to process concurrently\n";
}

ConcurrentStep::~ConcurrentStep() {
	for (auto it = steps.begin(); it != steps.end(); ++it) {
		delete *it;
	}
	steps.clear();
	delete(driveStep);
}

bool ConcurrentStep::Run(std::shared_ptr<World> world) {
	bool finished = true;
	for (auto step : steps) {
		std::cout << "ConcurrentStep running: " << step << "\n";
		finished &= step->Run(world);
	}
	return finished;
}

CrabInfo ConcurrentStep::GetCrabInfo() {
	return steps.front()->GetCrabInfo();
}


bool WrappedStep::Run(std::shared_ptr<World> world) {
	if (!finished) {
		finished = step->Run(world);
	}
	return finished;
}
