#include "subsystems/GateSubsystem.h"
#include "Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc/smartdashboard/SmartDashboard.h"


GateSubsystem::GateSubsystem(std::unique_ptr<GateIO> gatIO) : io(std::move(gatIO)) {
    SetName("GateSubsystem");
}

frc2::CommandPtr GateSubsystem::RunGateCommand() {
    return frc2::cmd::Run([this] {SetGateVolts(GateConstants::kGateVolts);}, {this})
    .FinallyDo([this] {
        SetGateVolts(0_V);
    });
}

frc2::CommandPtr GateSubsystem::RunOutake() {
    return frc2::cmd::Run([this] {SetGateVolts(-6_V);}, {this})
    .FinallyDo([this] {
        SetGateVolts(0_V);
    });
}

void GateSubsystem::Periodic() {
    io -> UpdateInputs(inputs);
    frc::SmartDashboard::PutNumber("gate rollers", inputs.gateCurrent.value());
    frc::SmartDashboard::PutNumber("Gate volts", inputs.gateVolts.value());
}