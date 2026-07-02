#include "subsystems/GateSubsystem.h"
#include "Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

GateSubsystem::GateSubsystem(std::unique_ptr<GateIO> gatIO) : io(std::move(gatIO)) {
    SetName("GateSubsystem");
}

frc2::CommandPtr GateSubsystem::RunGateCommand() {
    return frc2::cmd::Run([this] {SetGateVolts(GateConstants::kGateVolts);}, {this})
    .FinallyDo([this] {
        SetGateVolts(0_V);
    });
}

void GateSubsystem::Periodic() {
    io -> UpdateInputs(inputs);
}