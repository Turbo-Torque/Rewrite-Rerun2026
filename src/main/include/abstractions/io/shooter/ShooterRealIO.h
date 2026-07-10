#pragma once

#include <cmath>

#include "abstractions/io/shooter/ShooterIO.h"
#include "Constants.h"

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"

#include "units/angular_velocity.h"
#include "units/current.h"

class ShooterRealIO : public ShooterIO {
public:
    ShooterRealIO() {
        ConfigShooterMotor();
    }

    void UpdateInputs(ShooterIOInputs& inputs) override {
        auto rpm = leftShooterMotor.GetVelocity().GetValue();

        inputs.shooterRPM = units::revolutions_per_minute_t{rpm};
        inputs.shooterCurrent = leftShooterMotor.GetTorqueCurrent().GetValue();

        inputs.atRotations =
            std::abs((inputs.shooterRPM - targetRPM).value()) < 100.0;
    }

    void SetShooterRPM(units::revolutions_per_minute_t rpm) override {
        targetRPM = rpm;

        auto rpm = units::revolutions_per_minute_t{rpm};

        leftShooterMotor.SetControl(
            velocityRequest.WithVelocity(rpm));
    }

private:
    ctre::phoenix6::hardware::TalonFX leftShooterMotor{
        ShooterConstants::kLeftShooterMotorPort};

    ctre::phoenix6::hardware::TalonFX rightShooterMotor{
        ShooterConstants::kRightShooterMotorPort};

    ctre::phoenix6::controls::VelocityVoltage velocityRequest{0_tps};

    units::revolutions_per_minute_t targetRPM{0_rpm};

    void ConfigShooterMotor() {
        ctre::phoenix6::configs::TalonFXConfiguration config{};

        config.MotorOutput.NeutralMode =
            ctre::phoenix6::signals::NeutralModeValue::Coast;

        config.CurrentLimits.SupplyCurrentLimit = 45_A;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = 0.12;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.2;
        config.Slot0.kV = 0.12;

        leftShooterMotor.GetConfigurator().Apply(config);
        rightShooterMotor.GetConfigurator().Apply(config);

        leftShooterMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        rightShooterMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast); 

        rightShooterMotor.SetControl(
            ctre::phoenix6::controls::Follower{
                leftShooterMotor.GetDeviceID(),
                false
            });
    }
};