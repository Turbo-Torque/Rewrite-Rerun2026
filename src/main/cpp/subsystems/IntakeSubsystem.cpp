#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <string>


IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> intakeIo) : io(std::move(intakeIo)) {
    SetName("IntakeSubsystem");
} 

frc2::CommandPtr IntakeSubsystem::PivotAndRunIntakeCommand() {
    return GuardedByReset(
        frc2::cmd::Run([this] {SetIntakeSetpoint(IntakeConstants::kIntakeDown + pivotOffset);}, {this})
        .Until([this]() { return AtPivotSetpoint(IntakeConstants::kIntakeDown + pivotOffset); })
        .AndThen(frc2::cmd::Run([this] {SetIntakeVoltage(IntakeConstants::kIntakeVolts);}, {this}))
        .FinallyDo([this] {
            SetIntakeVoltage(0_V);
            SetIntakeSetpoint(IntakeConstants::kIntakeHalfway + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::PivotAndRunOuttakeCommand() {
    return GuardedByReset(
        frc2::cmd::Run([this] {SetIntakeSetpoint(IntakeConstants::kIntakeDown + pivotOffset);}, {this})
        .Until([this]() { return AtPivotSetpoint(IntakeConstants::kIntakeDown + pivotOffset);})
        .AndThen(frc2::cmd::Run([this] {SetIntakeVoltage(IntakeConstants::kOuttakeVolts);}, {this}))
        .FinallyDo([this] {
            SetIntakeVoltage(0_V);
            SetIntakeSetpoint(IntakeConstants::kIntakeHalfway + pivotOffset);
        })
    );
}


frc2::CommandPtr IntakeSubsystem::AgitateCommand() {
    return GuardedByReset(
        frc2::cmd::Run([this] {Agitate(IntakeConstants::kIntakeAgitate + pivotOffset);}, {this})
        .AndThen(frc2::cmd::Run([this] {SetAgitateVolts(IntakeConstants::kIntakeAgitateVolts);}, {this}))
        .FinallyDo([this] {
            SetAgitateVolts(0_V);
            Agitate(IntakeConstants::kIntakeHalfway + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::SupplyPivotVoltsCommand() {
    // Not guarded: this IS the reset/recalibration routine, gating it on itself would deadlock it.
    return frc2::cmd::Run([this] {SetPivotVolts(IntakeConstants::kPivotVolts);}, {this})
    .FinallyDo([this] {
        SetPivotVolts(0_V);
        newIntakePivot = inputs.position;  // real pivot position, not roller RPM
        pivotOffset = newIntakePivot - IntakeConstants::kIntakeDown;  // drift from where "down" should be
    });
}

frc2::CommandPtr IntakeSubsystem::PivotVolts() {
    return GuardedByReset(
        frc2::cmd::Run([this] {SetPivotVolts(IntakeConstants::kPivotVolts);}, {this})
        .FinallyDo([this] {
            SetPivotVolts(0_V);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::SetIntakeToUp() {
    return GuardedByReset(
        frc2::cmd::Run([this] {SetIntakeSetpoint(0.0 + pivotOffset);}, {this})
        .FinallyDo([this] {
            SetIntakeSetpoint(0.0 + pivotOffset);
        })
    );
}

frc2::CommandPtr IntakeSubsystem::GuardedByReset(frc2::CommandPtr&& command) {
    return frc2::cmd::Either(std::move(command), frc2::cmd::None(), [this] { return !resetIntake; });
}

bool IntakeSubsystem::IntakeNeedHopper() {
    if (inputs.intakeCurrent >= 50_A && inputs.rotations < 3000.0) {
        return true;
    } else {
        return false;
    }
}

bool IntakeSubsystem::AgitateRollers() {
    return true;
}

bool IntakeSubsystem::ResetIntake() {
    return resetIntake;
}

frc2::CommandPtr IntakeSubsystem::SetResetIntake() {
    // FIXED: this only flips a flag, no hardware access — requiring {this} made it illegally
    // overlap with SupplyPivotVoltsCommand() (also requires IntakeSubsystem) in their .AlongWith(...)
    // binding, throwing "Multiple commands in a parallel group cannot require the same subsystems"
    // at startup (bindings get built during RobotContainer's constructor).
    return frc2::cmd::RunOnce([this] {resetIntake = true;})
    .FinallyDo([this] {resetIntake = false;});
}


void IntakeSubsystem::Periodic() {
    io -> UpdateInputs(inputs);

    // ADDED: pivotOffset so these thresholds stay correct after SupplyPivotVoltsCommand() recalibrates
    std::string intakeState;
    if (inputs.position >  (IntakeConstants::kIntakeDown + pivotOffset - 3) && inputs.position <= (IntakeConstants::kIntakeDown + pivotOffset + 5)) {
        intakeState = "Intake Down";
    } else if (inputs.position >  (IntakeConstants::kIntakeHalfway + pivotOffset - 3) && inputs.position <= (IntakeConstants::kIntakeHalfway + pivotOffset + 5)) {
        intakeState = "Intake Halfway";
    } else if (inputs.position >  (IntakeConstants::kIntakeAgitate + pivotOffset - 2) && inputs.position <= (IntakeConstants::kIntakeAgitate + pivotOffset + 5)) {
        intakeState = "Intake Agitate";
    } else {
        intakeState = "Intake Up";
    }

    
    frc::SmartDashboard::PutNumber("Intake Pose", inputs.position);
    frc::SmartDashboard::PutNumber("Intake Volts", inputs.intakeVolts.value());
    frc::SmartDashboard::PutBoolean("intake setpoint", inputs.pivotAtSetpoint);
    frc::SmartDashboard::PutNumber("intake current", inputs.intakeCurrent.value());
    frc::SmartDashboard::PutNumber("intake rollers rpm", inputs.rotations);
    frc::SmartDashboard::PutString("Intake State", intakeState);
    frc::SmartDashboard::PutBoolean("Intake Roller Stall", IntakeNeedHopper());
    // REMOVED (by request): this ran every tick unconditionally, regardless of which command (if
    // any) was active, and fought PivotAndRunOuttakeCommand()'s FinallyDo() — the pivot sits in
    // this exact position window right after outtake ends, so this kept re-enabling forward intake
    // voltage every cycle instead of letting the roller stay at 0V / the pivot settle at halfway.
    // PivotAndRunIntakeCommand() already runs the rollers itself when intentionally going down.
    // if (AgitateRollers()) {
    //     SetIntakeVoltage(IntakeConstants::kIntakeAgitateVolts);
    // } else {
    //     SetIntakeVoltage(0_V);
    // }


}
