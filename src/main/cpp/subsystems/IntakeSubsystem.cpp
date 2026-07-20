#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"


IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> intakeIo) : io(std::move(intakeIo)) {
    SetName("IntakeSubsystem");
} 

frc2::CommandPtr IntakeSubsystem::PivotAndRunIntakeCommand() {
    return frc2::cmd::Run([this] {SetIntakePivot(true);}, {this})
    .Until([this]() { return inputs.pivotAtSetpoint; })
    .AndThen(frc2::cmd::Run([this] {SetIntakeVoltage(IntakeConstants::kIntakeVolts);}, {this}))
    .FinallyDo([this] {
        SetIntakeVoltage(0_V);
        SetIntakePivot(false);
    });
}

void IntakeSubsystem::ToggleIntake() {
    intakeDeployed = !intakeDeployed;

    if(!intakeDeployed) {
        SetIntakeVoltage(0_V);
        SetIntakePivot(false);
    }
}

void IntakeSubsystem::Periodic() {
    io -> UpdateInputs(inputs);

    frc::SmartDashboard::PutNumber("Intake Pose", inputs.position);
    frc::SmartDashboard::PutNumber("Intake Volts", inputs.intakeVolts.value());


    if (intakeDeployed) {
        SetIntakePivot(true);
        if (inputs.pivotAtSetpoint) {
            SetIntakeVoltage(IntakeConstants::kIntakeVolts);
        }
    }
}
