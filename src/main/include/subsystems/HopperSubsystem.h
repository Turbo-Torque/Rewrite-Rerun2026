#pragma once
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/hopper/HopperIO.h"

class HopperSubsystem : public frc2::SubsystemBase{
    public:
        HopperSubsystem(std::unique_ptr<HopperIO> io);
        void SetHopperVoltage(units::volt_t volts) { io -> SetHopperVolts(volts);}

        frc2::CommandPtr RunHopperCommand();

        void FeedRequsted(bool request);
        void StopHopper() { SetHopperVoltage(0_V);};
        void Periodic() override;

    private:
        std::unique_ptr<HopperIO> io;
        HopperIOInputs inputs;
};