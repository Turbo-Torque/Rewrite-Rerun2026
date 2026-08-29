#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"


IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> intakeIo) : io(std::move(intakeIo)) {
    SetName("IntakeSubsystem");
} 

frc2::CommandPtr IntakeSubsystem::PivotAndRunIntakeCommand() {
    return frc2::cmd::Run([this] {SetIntakeSetpoint(IntakeConstants::kIntakeDown);}, {this})
    .Until([this]() { return inputs.pivotAtSetpoint; })
    .AndThen(frc2::cmd::Run([this] {SetIntakeVoltage(IntakeConstants::kIntakeVolts);}, {this}))
    .FinallyDo([this] {
        SetIntakeVoltage(0_V);
        SetIntakeSetpoint(IntakeConstants::kIntakeUp);
    });
}


frc2::CommandPtr IntakeSubsystem::AgitateCommand() {
    return frc2::cmd::Run([this] {Agitate(IntakeConstants::kIntakeAgitate);}, {this})
    .AndThen(frc2::cmd::Run([this] {SetIntakeVoltage(IntakeConstants::kIntakeVolts);}, {this}))
    .FinallyDo([this] {
        Agitate(IntakeConstants::kIntakeHalfway);
    });
}

bool IntakeSubsystem::IntakeNeedHopper() {
    if (inputs.pivotCurrent >= 40_A && inputs.rotations <= 0 && inputs.pivotAtSetpoint) {
        return true;
    }
    return false;
}

void IntakeSubsystem::Periodic() {
    io -> UpdateInputs(inputs);

    frc::SmartDashboard::PutNumber("Intake Pose", inputs.position);
    frc::SmartDashboard::PutNumber("Intake Volts", inputs.intakeVolts.value());
    frc::SmartDashboard::PutBoolean("intake setpoint", inputs.pivotAtSetpoint);
    frc::SmartDashboard::PutNumber("intake current", inputs.intakeCurrent.value());
    frc::SmartDashboard::PutNumber("intake rollers rpm", inputs.rotations);
    if (inputs.pivotAtSetpoint && (inputs.position < 0) && (inputs.position > 0) ) {
        SetIntakeVoltage(IntakeConstants::kIntakeVolts);
    } else {
        SetIntakeVoltage(0_V);
    }


    // if (inputs.pivotCurrent < 20_A) {
    //     (IntakeConstants::kIntakeAgitateVolts);
    // } else {
    //     SetIntakeVoltage(0_V)
    // }


}
