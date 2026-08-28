#pragma once
#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/shooter/ShooterIO.h"

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
        frc2::CommandPtr Laser();
        frc2::CommandPtr RunShooterCommand2(units::revolutions_per_minute_t rpm, double hoodAngle);
        frc2::CommandPtr RunShooterCommand3();
        frc2::CommandPtr SetShooterState(double targeRPM, double targetHoodAngle);


        bool IsNearState();
        void Periodic() override;

    private:
        std::unique_ptr<ShooterIO> io;
        ShooterIOInputs inputs;
        double targetRPM = 0.0;
        double targetHoodAngle = 0.0;
};