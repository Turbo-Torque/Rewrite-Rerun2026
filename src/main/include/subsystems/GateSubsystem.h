#pragma once
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/gate/GateIO.h"

class GateSubsystem : public frc2::SubsystemBase {
    public:
        GateSubsystem(std::unique_ptr<GateIO> io);
        void SetGateVolts(units::volt_t volts) {io -> SetGateVolts(volts);};

        frc2::CommandPtr RunGateCommand();

        void Periodic() override;
    private:
        std::unique_ptr<GateIO> io;
        GateIOInputs inputs;

};