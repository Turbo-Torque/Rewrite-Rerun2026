#pragma once

#include "abstractions/io/hopper/HopperIO.h"
#include "rev/SparkMax.h"
#include "Constants.h"
#include "rev/ClosedLoopTypes.h"
#include "rev/ConfigureTypes.h"
#include "rev/SparkLowLevel.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"





class HopperRealIO : public HopperIO {
    public:
        HopperRealIO() {
            ConfigHopperMotor();
        }

        void UpdateInputs(HopperIOInputs& inputs) override {
            inputs.hopperVolts = units::volt_t{hopperMotor.GetAppliedOutput() * hopperMotor.GetBusVoltage()};
        }

        void SetHopperVolts(units::volt_t volts) override{
            hopperMotor.SetVoltage(volts);
        };

    private:
        rev::spark::SparkMax hopperMotor{HopperConstants::kHopperMotorPort, rev::spark::SparkLowLevel::MotorType::kBrushless};
        void ConfigHopperMotor() {
            rev::spark::SparkMaxConfig config;

            config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
            config.SmartCurrentLimit(35, 50);
            config.VoltageCompensation(RobotConstants::kStartVoltage);
            config.OpenLoopRampRate(0.08);

            hopperMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
        };
};