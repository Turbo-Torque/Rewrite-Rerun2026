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
        ConfigHoodMotor();
    }

    void UpdateInputs(ShooterIOInputs& inputs) override {
        inputs.shooterRPM = units::revolutions_per_minute_t{leftShooterMotor.GetVelocity().GetValue()};
        inputs.shooterCurrent = units::ampere_t{leftShooterMotor.GetTorqueCurrent().GetValue()};
        inputs.hoodPosition = hoodMotor.GetEncoder().GetPosition();
        inputs.hoodSetPoint = hoodMotor.GetClosedLoopController().GetSetpoint();

        inputs.atRotations =
            std::abs((inputs.shooterRPM - targetRPM).value()) < 100.0;
        if ((inputs.hoodPosition < 57.0) && (inputs.hoodPosition > 50.0)) {
                inputs.hoodAtSetpoint = true;
            } 
    }

    void SetShooterRPM(units::revolutions_per_minute_t rpm) override {
        leftShooterMotor.SetControl(
            velocityRequest.WithVelocity(rpm));
    }

    void SetHoodAngle(bool deployed) {
        double hoodSetpoint;
        if (deployed) {
            hoodSetpoint = ShooterConstants::kHoodUp;

        } else {
            hoodSetpoint = ShooterConstants::kHoodDown;
        }
        hoodMotor.GetClosedLoopController().SetSetpoint(hoodSetpoint, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot0, ShooterConstants::kFFHood);
    }

    void SetHoodSetpoint(double rot) {
        hoodMotor.GetClosedLoopController().SetSetpoint(rot, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot0, ShooterConstants::kFFHood);
    }

private:
    ctre::phoenix6::hardware::TalonFX leftShooterMotor{
        ShooterConstants::kLeftShooterMotorPort};

    ctre::phoenix6::hardware::TalonFX rightShooterMotor{
        ShooterConstants::kRightShooterMotorPort};

    rev::spark::SparkMax hoodMotor{ShooterConstants::kHoodMotorPort, rev::spark::SparkLowLevel::MotorType::kBrushless};

    ctre::phoenix6::controls::VelocityVoltage velocityRequest{0_tps};

    units::revolutions_per_minute_t targetRPM{0_rpm};



    void ConfigShooterMotor() {
        ctre::phoenix6::configs::TalonFXConfiguration config{};

        config.MotorOutput.NeutralMode =
            ctre::phoenix6::signals::NeutralModeValue::Coast;

        config.CurrentLimits.SupplyCurrentLimit = 45_A;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

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
                true
            }
        );
    }

    void ConfigHoodMotor() {
        rev::spark::SparkMaxConfig config;
        config.closedLoop.P(0.04, rev::spark::kSlot0);
        config.closedLoop.I(0.001, rev::spark::kSlot0);
        config.closedLoop.D(0.002, rev::spark::kSlot0);

        config.closedLoop.P(0.02, rev::spark::kSlot1);
        config.closedLoop.I(0.0, rev::spark::kSlot1);
        config.closedLoop.D(0.002, rev::spark::kSlot1);

        config.SmartCurrentLimit(35, 50);
        config.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);
        config.OpenLoopRampRate(0.1);
        config.closedLoop.AllowedClosedLoopError(0.20);
        config.Inverted(true);

        hoodMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
        hoodMotor.GetEncoder().SetPosition(0.0);
    }

};