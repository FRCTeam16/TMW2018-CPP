
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
		std::cout << frc::Timer::GetFPGATimestamp() << " DoClimb:Forward | First Run\n";
	}

	if (IsTimeElapsed() && Robot::elevator->InPosition()) {
		Robot::elevator->HoldClimb();
		finished = true;
		std::cout << frc::Timer::GetFPGATimestamp() << " DoClimb:Forward | COMPLETE\n";
	}
}

void DoClimb::Reverse() {
	if (IsFirstRun()) {
        Robot::elevator->UnholdClimb();
        Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kClimb);
        std::cout << frc::Timer::GetFPGATimestamp() << " DoClimb:Reverse | First Run\n";
    }

    if (IsTimeElapsed() && Robot::elevator->InPosition()) {
        finished = true;
        std::cout << frc::Timer::GetFPGATimestamp() << " DoClimb:Reverse | COMPLETE\n";
    }
}

bool DoClimb::IsFinished() {
	return finished;
}
