#include <Climb/DoPositionElevatorAndMastForClimb.h>
#include <Robot.h>
#include <Subsystems/Elevator.h>
#include <Subsystems/Mast.h>

DoPositionElevatorAndMastForClimb::DoPositionElevatorAndMastForClimb() {
}

DoPositionElevatorAndMastForClimb::~DoPositionElevatorAndMastForClimb() {
}

void DoPositionElevatorAndMastForClimb::Forward() {
	if (StateTransition::IsFirstRun()) {
		start = frc::Timer::GetFPGATimestamp();
		Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kFloor);
		Robot::mast->SetMastPosition(Mast::MastPosition::kVertical);
	}

	if (!floorReached && !elevateComplete && TimeElapsed(0.1) && Robot::mast->InPosition() && Robot::elevator->InPosition()) {
		Robot::elevator->ShiftLow();
		Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kClimb);
		Robot::intake->SetExtendSolenoidState(true);
		start = frc::Timer::GetFPGATimestamp();
		floorReached = true;
	} else if (floorReached && !elevateComplete && TimeElapsed(0.1) && Robot::elevator->InPosition()) {
		start = frc::Timer::GetFPGATimestamp();
		elevateComplete = true;
	} else if (floorReached && elevateComplete && TimeElapsed(0.1) && Robot::elevator->InPosition()) {
		complete = true;
	}
}

bool DoPositionElevatorAndMastForClimb::TimeElapsed(double elapsed) {
	return (frc::Timer::GetFPGATimestamp() - start) > elapsed;
}


void DoPositionElevatorAndMastForClimb::Reverse() {
	if (StateTransition::IsFirstRun()) {
		Robot::elevator->SetElevatorPosition(Elevator::ElevatorPosition::kFloor);
	}
	if (IsTimeElapsed() && Robot::elevator->InPosition()) {
		Robot::elevator->ShiftHigh();
		complete = true;
	}
}

bool DoPositionElevatorAndMastForClimb::IsFinished() {
	return complete;
}
