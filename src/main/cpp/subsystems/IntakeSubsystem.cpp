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
        SetIntakeSetpoint(IntakeConstants::kIntakeHalfway);
    });
}

frc2::CommandPtr IntakeSubsystem::PivotAndRunOuttakeCommand() {
    return frc2::cmd::Run([this] {SetIntakeSetpoint(IntakeConstants::kIntakeDown);}, {this})
    .Until([this]() { return inputs.pivotAtSetpoint; })
    .AndThen(frc2::cmd::Run([this] {SetIntakeVoltage(IntakeConstants::kOuttakeVolts);}, {this}))
    .FinallyDo([this] {
        SetIntakeVoltage(0_V);
        SetIntakeSetpoint(IntakeConstants::kIntakeHalfway);
    });
}


frc2::CommandPtr IntakeSubsystem::AgitateCommand() {
    return frc2::cmd::Run([this] {Agitate(IntakeConstants::kIntakeAgitate);}, {this})
    .AndThen(frc2::cmd::Run([this] {SetAgitateVolts(IntakeConstants::kIntakeAgitateVolts);}, {this}))
    .FinallyDo([this] {
        SetAgitateVolts(0_V);
        Agitate(IntakeConstants::kIntakeHalfway);
    });
}

bool IntakeSubsystem::IntakeNeedHopper() {
    if (inputs.pivotCurrent >= 50_A && inputs.rotations < 100 && inputs.pivotAtSetpoint) {
        return true;
    }
    return false;
}

bool IntakeSubsystem::AgitateRollers() {
    return true;
}

void IntakeSubsystem::Periodic() {
    io -> UpdateInputs(inputs);

    frc::SmartDashboard::PutNumber("Intake Pose", inputs.position);
    frc::SmartDashboard::PutNumber("Intake Volts", inputs.intakeVolts.value());
    frc::SmartDashboard::PutBoolean("intake setpoint", inputs.pivotAtSetpoint);
    frc::SmartDashboard::PutNumber("intake current", inputs.intakeCurrent.value());
    frc::SmartDashboard::PutNumber("intake rollers rpm", inputs.rotations);
    if (inputs.pivotAtSetpoint && (inputs.position > 55) && (inputs.position <= 62) ) {
        SetIntakeVoltage(IntakeConstants::kIntakeVolts);
    } else {
        SetIntakeVoltage(0_V);
    }
    // if (AgitateRollers()) {
    //     SetIntakeVoltage(IntakeConstants::kIntakeAgitateVolts);
    // } else {
    //     SetIntakeVoltage(0_V);
    // }


}
