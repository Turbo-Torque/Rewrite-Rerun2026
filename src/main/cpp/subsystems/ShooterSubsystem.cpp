#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

ShooterSubsystem::ShooterSubsystem(std::unique_ptr<ShooterIO> shooterIO) : io(std::move(shooterIO)) {
    SetName("ShooterSubsystem");
}

frc2::CommandPtr ShooterSubsystem::RunShooterCommand() {
    return frc2::cmd::Run([this] {SetShooterRPM(ShooterConstants::kShooterRPM);}, {this})
    .FinallyDo([this] {
        SetShooterRPM(0_rpm);
    });
}

bool ShooterSubsystem::IsNearState() {
    return inputs.atRotations;
}


void ShooterSubsystem::Periodic() {
    io -> UpdateInputs(inputs);
}