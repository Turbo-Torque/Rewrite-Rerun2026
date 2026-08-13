#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"


ShooterSubsystem::ShooterSubsystem(std::unique_ptr<ShooterIO> shooterIO) : io(std::move(shooterIO)) {
    SetName("ShooterSubsystem");
}

frc2::CommandPtr ShooterSubsystem::RunShooterCommand(units::revolutions_per_minute_t rpm) {
    return frc2::cmd::Run([this, rpm] {SetShooterRPM(rpm); {SetHoodSetpoint(ShooterConstants::kHoodDown);}}, {this})
    .FinallyDo([this] {
        CoastOut();
        SetHoodSetpoint(ShooterConstants::kHoodDown);
        
    });
}


frc2::CommandPtr ShooterSubsystem::RunHoodCommand(double hoodAngle) {
    return frc2::cmd::Run([this, hoodAngle] {SetHoodSetpoint(hoodAngle);}, {this})
    .FinallyDo([this] {
        SetHoodSetpoint(ShooterConstants::kHoodDown);
        });
    
frc2::CommandPtr ShooterSubsystem::RunLaseringCommand() {
    return frc2::cmd::Run([this] {SetShooterRPM(ShooterConstants::kShooterRPM); {SetHoodSetpoint(ShooterConstants::kHoodUp);}}, {this})
    .FinallyDo([this] {
        CoastOut();
        RunHoodCommand(ShooterConstants::kHoodDown);
    });
}

bool ShooterSubsystem::IsNearState() {
    return inputs.atRotations;
}

frc2::CommandPtr ShooterSubsystem::GetRegressionShot(units::meter_t distance) {
    auto it = rpmTable.upper_bound(distance);

    units::revolutions_per_minute_t rpm;

    if (it == rpmTable.begin()) {
        rpm = rpmTable.begin()->second;
    } else {
        --it;
        rpm = it->second;
    }

    

}

// void ShooterSubsystem::SetHoodAngleGoal(units::degree_t angle) {
//     SetHoodSetpoint(angle.value());
// }


void ShooterSubsystem::Periodic() {
    io -> UpdateInputs(inputs);
    frc::SmartDashboard::PutNumber("Shooter Rpm ", inputs.shooterRPM.value());
    frc::SmartDashboard::PutNumber("Shooter Setpoint", inputs.shooterRPMsetpoint.value());
    frc::SmartDashboard::PutNumber("Shooter Volts", inputs.shooterCurrent.value());
    frc::SmartDashboard::PutNumber("Hood Angle", inputs.hoodPosition);
    frc::SmartDashboard::PutNumber("Hood Setpoint", inputs.hoodSetPoint);
    frc::SmartDashboard::PutNumber("Hood Current", inputs.hoodCurrent.value());

    frc::SmartDashboard::GetNumber("1Shooter Rpm", inputs.shooterRPM.value());
    frc::SmartDashboard::GetNumber("1Shooter Setpoint", inputs.shooterRPMsetpoint.value());
    frc::SmartDashboard::GetNumber("1Shooter Volts", inputs.shooterCurrent.value());
    frc::SmartDashboard::GetNumber("1Hood Angle", inputs.hoodPosition);
    frc::SmartDashboard::GetNumber("1Hood Setpoint", inputs.hoodSetPoint);
}