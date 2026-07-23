#pragma once

#include <cmath>

#include "abstractions/io/shooter/ShooterIO.h"
#include "Constants.h"

#include "ctre/phoenix6/TalonFX.hpp"
#include <ctre/phoenix6/CANcoder.hpp>
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/controls/CoastOut.hpp"

#include "units/angular_velocity.h"
#include "units/current.h"

class ShooterRealIO : public ShooterIO {
public:
    ShooterRealIO() {
        ConfigShooterMotor();
        ConfigHoodMotor();
    }

    void UpdateInputs(ShooterIOInputs& inputs) override {
        units::degree_t hoodPosition = hoodCAN.GetAbsolutePosition().GetValue();

        inputs.shooterRPM = units::revolutions_per_minute_t{leftShooterMotor.GetVelocity().GetValue()};
        inputs.shooterCurrent = units::ampere_t{leftShooterMotor.GetTorqueCurrent().GetValue()};
        inputs.hoodPosition = hoodPosition.value();
        inputs.hoodSetPoint = hoodPID.GetSetpoint();
        inputs.hoodCurrent = units::ampere_t{hoodMotor.GetOutputCurrent()};

        units::volt_t hoodOut = units::volt_t{hoodPID.Calculate(hoodPosition.value())};

        hoodMotor.SetVoltage(hoodOut);

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

    void CoastOut() override {
        leftShooterMotor.SetControl(coastRequest);
    }


    void SetHoodSetpoint(double rot) {
        hoodPID.SetSetpoint(rot);
    }

private:
    ctre::phoenix6::hardware::TalonFX leftShooterMotor{
        ShooterConstants::kLeftShooterMotorPort};

    ctre::phoenix6::hardware::TalonFX rightShooterMotor{
        ShooterConstants::kRightShooterMotorPort};

    rev::spark::SparkMax hoodMotor{ShooterConstants::kHoodMotor, rev::spark::SparkLowLevel::MotorType::kBrushless};

	ctre::phoenix6::hardware::CANcoder hoodCAN{ShooterConstants::kHoodCAN};

    ctre::phoenix6::controls::VelocityVoltage velocityRequest{0_tps};

    ctre::phoenix6::controls::CoastOut coastRequest;

    units::revolutions_per_minute_t targetRPM{0_rpm};

    frc::PIDController hoodPID{0.12,0,0};


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

        config.SmartCurrentLimit(20, 30);
        config.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);
        config.OpenLoopRampRate(0.1);
        config.closedLoop.AllowedClosedLoopError(0.20);

        hoodMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
        hoodMotor.GetEncoder().SetPosition(0.0);

    }

};