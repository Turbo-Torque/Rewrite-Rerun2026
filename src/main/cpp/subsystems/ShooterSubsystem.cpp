#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"


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

void ShooterSubsystem::SetHoodAngleGoal(units::degree_t angle) {
    double rotations = (angle.value() / 360) * ShooterConstants::kHoodGearRatio;
    SetHoodSetpoint(rotations);
}


void ShooterSubsystem::Periodic() {
    io -> UpdateInputs(inputs);
    frc::SmartDashboard::PutNumber("Shooter Rpm ", inputs.shooterRPM.value());
    frc::SmartDashboard::PutNumber("Shooter Setpoint", inputs.shooterRPMsetpoint.value());
    frc::SmartDashboard::PutNumber("Shooter Volts", inputs.shooterCurrent.value());
    frc::SmartDashboard::PutNumber("Hood Angle", inputs.hoodPosition);
    frc::SmartDashboard::PutNumber("Hood Setpoint", inputs.hoodSetPoint);

    frc::SmartDashboard::GetNumber("1Shooter Rpm", inputs.shooterRPM.value());
    frc::SmartDashboard::GetNumber("1Shooter Setpoint", inputs.shooterRPMsetpoint.value());
    frc::SmartDashboard::GetNumber("1Shooter Volts", inputs.shooterCurrent.value());
    frc::SmartDashboard::GetNumber("1Hood Angle", inputs.hoodPosition);
    frc::SmartDashboard::GetNumber("1Hood Setpoint", inputs.hoodSetPoint);
}