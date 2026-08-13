#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"


ShooterSubsystem::ShooterSubsystem(std::unique_ptr<ShooterIO> shooterIO) : io(std::move(shooterIO)) {
    SetName("ShooterSubsystem");

    // put smartdashboard (controls rpm)
    frc::SmartDashboard::PutNumber("Put Shooter RPM", 0.0);
    frc::SmartDashboard::PutNumber("Put Hood Angle", 0.0);

}

frc2::CommandPtr ShooterSubsystem::RunShooterCommand() {
    return frc2::cmd::Run([this] {SetShooterRPM(ShooterConstants::kShooterRPM);
        SetHoodSetpoint(ShooterConstants::kHoodUp);}, {this})
    .FinallyDo([this] {
        CoastOut();
        SetHoodSetpoint(ShooterConstants::kHoodDown);
    });
}

frc2::CommandPtr ShooterSubsystem::TestShooter() {
    return frc2::cmd::Run([this] {SetShooterRPM(units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Put Shooter RPM", 0.0)});
        SetHoodSetpoint(frc::SmartDashboard::GetNumber("Put Hood Angle", 0.0));}, {this})
    .FinallyDo([this] {
        CoastOut();
        SetHoodSetpoint(ShooterConstants::kHoodDown);
    });
}

bool ShooterSubsystem::IsNearState() {
    return inputs.atRotations;
}

// void ShooterSubsystem::SetHoodAngleGoal(units::degree_t angle) {
//     SetHoodSetpoint(angle.value());
// }


void ShooterSubsystem::Periodic() {
    io -> UpdateInputs(inputs);

    // display rpm
    frc::SmartDashboard::PutNumber("Shooter Rpm ", inputs.shooterRPM.value());
    frc::SmartDashboard::PutNumber("Shooter Setpoint", inputs.shooterRPMsetpoint.value());
    frc::SmartDashboard::PutNumber("Shooter Volts", inputs.shooterCurrent.value());
    frc::SmartDashboard::PutNumber("Hood Angle", inputs.hoodPosition);
    frc::SmartDashboard::PutNumber("Hood Setpoint", inputs.hoodSetPoint);
    frc::SmartDashboard::PutNumber("Hood Current", inputs.hoodCurrent.value());

    

}