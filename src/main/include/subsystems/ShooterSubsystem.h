#pragma once
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/shooter/ShooterIO.h"
#include "turbolib/structure/ShooterSetpoint.hpp"

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
        ShooterSubsystem(std::unique_ptr<ShooterIO> io);
        void SetShooterRPM(units::revolutions_per_minute_t rpm) {io -> SetShooterRPM(rpm);}
        void SetHoodSetpoint(double rot) {io ->SetHoodSetpoint(rot);}
        void CoastOut() {io -> CoastOut();}
        // void SetHoodAngleGoal(units::degree_t angle);

        frc2::CommandPtr RunShooterCommand(units::revolutions_per_minute_t rpm);
        frc2::CommandPtr RunHoodCommand(double hoodAngle);
        frc2::CommandPtr RunLaseringCommand();
        bool IsNearState();
        void Periodic() override;

        frc2::CommandPtr GetRegressionShot(units::meter_t distance);

    private:
        std::unique_ptr<ShooterIO> io;
        ShooterIOInputs inputs;

    std::map<units::meter_t, units::revolutions_per_minute_t> rpmTable{
        {0.0_m, 3000_rpm}
    };
};