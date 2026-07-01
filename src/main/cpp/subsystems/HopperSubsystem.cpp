#include "subsystems/HopperSubsystem.h"
#include "Constants.h"

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

HopperSubsystem::HopperSubsystem(std::unique_ptr<HopperIO> hopperIO) : io(std::move(hopperIO)) {
    SetName("HopperSubsystem");
}

frc2::CommandPtr HopperSubsystem::RunHopperCommand() {
    return frc2::cmd::Run([this] {SetHopperVoltage(HopperConstants::kHopperInVolts);}, {this})
    .FinallyDo([this] {
        SetHopperVoltage(0_V);
    });
}

void HopperSubsystem::Periodic() {
    io -> UpdateInputs(inputs);
}