#pragma once
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/shooter/ShooterIO.h"

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
        ShooterSubsystem(std::unique_ptr<ShooterIO> io);
        void SetShooterRPM(units::revolutions_per_minute_t rpm) {io -> SetShooterRPM(rpm);}

        frc2::CommandPtr RunShooterCommand();

        bool IsNearState();
        void Periodic() override;

    private:
        std::unique_ptr<ShooterIO> io;
        ShooterIOInputs inputs;
};