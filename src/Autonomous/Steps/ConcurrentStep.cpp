
#include <iostream>
#include <Autonomous/Steps/ConcurrentStep.h>

ConcurrentStep::ConcurrentStep(std::initializer_list<Step*> stepList) {
	std::cout << "Loaded " << steps.size() << " steps to process concurrently\n";
	for (Step* step : stepList) {
		steps.push_back(new WrappedStep(step));
	}
}

ConcurrentStep::~ConcurrentStep() {
	for (auto it = steps.begin(); it != steps.end(); ++it) {
		delete *it;
	}
	steps.clear();
}

bool ConcurrentStep::Run(std::shared_ptr<World> world) {
	bool finished = true;
	for (auto step : steps) {
		std::cout << "ConcurrentStep running: " << step << "\n";
		finished &= step->Run(world);
	}
	std::cout << "Concurrent::Run complete with " << finished << "\n\n";
	return finished;
}


const CrabInfo* ConcurrentStep::GetCrabInfo() {
	WrappedStep *step = steps.front();
	const CrabInfo* crabInfo = step->GetStep()->GetCrabInfo();
	if (step->IsFinished()) {
		std::cout << "ConcurrentStep Stop\n";
		this->crab->Stop();
		return this->GetCrabInfo();
	} else {
		return step->GetStep()->GetCrabInfo();
	}
}


bool WrappedStep::Run(std::shared_ptr<World> world) {
	if (!finished) {
		finished = step->Run(world);
	}
	return finished;
}

bool WrappedStep::IsFinished() {
	return finished;
}
