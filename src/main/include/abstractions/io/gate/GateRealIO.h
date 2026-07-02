#pragma once
#include "abstractions/io/gate/GateIO.h"
#include "Constants.h"
#include "ctre/phoenix6/CANcoder.hpp"

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "units/current.h"
#include "units/voltage.h"

class GateRealIO : public GateIO {
    public:
        GateRealIO() {
            ConfigGateMotor();
        }

        void UpdateInputs(GateIOInputs& inputs) override{
            inputs.gateVolts = units::volt_t{leftGateMotor.GetMotorVoltage().GetValue()};
            inputs.gateCurrent = units::ampere_t{leftGateMotor.GetTorqueCurrent().GetValue()};
        }

        void SetGateVolts(units::volt_t volts) override {
            leftGateMotor.SetControl(voltageRequest.WithOutput(volts));
        }
    
    private:
        ctre::phoenix6::hardware::TalonFX leftGateMotor{GateConstants::kLeftGateMotorPort};
        ctre::phoenix6::hardware::TalonFX rightGateMotor{GateConstants::kRightGateMotorPort};
        ctre::phoenix6::controls::VoltageOut voltageRequest{0_V};

        void ConfigGateMotor() {
            ctre::phoenix6::configs::TalonFXConfiguration config{};
            config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            config.CurrentLimits.SupplyCurrentLimit = 45_A;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;

            leftGateMotor.GetConfigurator().Apply(config);
            rightGateMotor.GetConfigurator().Apply(config);

            rightGateMotor.SetControl(
                ctre::phoenix6::controls::Follower {leftGateMotor.GetDeviceID(), false}
            );
        }
};