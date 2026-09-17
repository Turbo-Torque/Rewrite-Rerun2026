#pragma once
#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/shooter/ShooterIO.h"
#include "units/length.h"
#include <wpi/interpolating_map.h>

class ShooterSubsystem : public frc2::SubsystemBase {
    public:
        ShooterSubsystem(std::unique_ptr<ShooterIO> io);
        void SetShooterRPM(units::revolutions_per_minute_t rpm) {io -> SetShooterRPM(rpm);}
        void SetHoodSetpoint(double rot) {io ->SetHoodSetpoint(rot);}
        void CoastOut() {io -> CoastOut();}
        // void SetHoodAngleGoal(units::degree_t angle);

        frc2::CommandPtr RunShooterCommand();
        frc2::CommandPtr RunHoodCommand(double hoodAngle);
        frc2::CommandPtr TestShooter();
        // frc2::CommandPtr Laser();
        frc2::CommandPtr Laser(std::function<units::meter_t()> getDistance);
        frc2::CommandPtr RunShooterCommand2();
        frc2::CommandPtr RunShooterCommand3();
        frc2::CommandPtr SetShooterState(double targeRPM, double targetHoodAngle);
        units::revolutions_per_minute_t GetShotRPM(units::meter_t distance) { return shotRegression[distance]; }

        bool IsNearState();
        void Periodic() override;

    private:
        std::unique_ptr<ShooterIO> io;
        ShooterIOInputs inputs;
        double targetRPM = 0.0;
        double targetHoodAngle = 0.0;
        wpi::interpolating_map<units::meter_t, units::revolutions_per_minute_t> shotRegression;
        // ADDED: last distance the Laser command computed, so Periodic() can publish it to Elastic
        units::meter_t lastLaserDistance{0_m};
};