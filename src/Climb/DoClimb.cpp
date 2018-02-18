
#include <Robot.h>
#include <Climb/DoClimb.h>
#include <Subsystems/Elevator.h>

DoClimb::DoClimb() {
}

DoClimb::~DoClimb() {
}

void DoClimb::Forward() {
	if (IsFirstRun()) {
		Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kDoClimb);
	}

	if (IsTimeElapsed() && Robot::elevator->InPosition()) {
		Robot::elevator->HoldClimb();
		finished = true;
	}
}

void DoClimb::Reverse() {
	if (IsFirstRun()) {
        Robot::elevator->UnholdClimb();
        Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kClimb);
    }

    if (IsTimeElapsed() && Robot::elevator->InPosition()) {
        finished = true;
    }
}

bool DoClimb::IsFinished() {
	return finished;
}
