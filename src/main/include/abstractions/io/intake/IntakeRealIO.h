#pragma once

#include "abstractions/io/intake/IntakeIO.h"
#include "Constants.h"
#include "ctre/phoenix6/CANcoder.hpp"

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "rev/ClosedLoopTypes.h"
#include "rev/ConfigureTypes.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include "units/current.h"
#include "units/voltage.h"

class IntakeRealIO : public IntakeIO {
    public:
        IntakeRealIO() {
            ConfigIntakeMotor();
            ConfigPivotMotor();
        };

        void UpdateInputs(IntakeIOInputs& inputs) override {
            inputs.position = pivotMotor.GetEncoder().GetPosition();
            inputs.setpoint = pivotMotor.GetClosedLoopController().GetSetpoint();
            inputs.pivotAtSetpoint = pivotMotor.GetClosedLoopController().IsAtSetpoint();
            inputs.intakeVolts = units::volt_t{intakeMotor.GetMotorVoltage().GetValue()};
            inputs.intakeCurrent = units::ampere_t{intakeMotor.GetTorqueCurrent().GetValue()};
        }

        void SetIntakeVolts(units::volt_t voltage) override{
            intakeMotor.SetControl(voltageRequest.WithOutput(voltage));
        }

        void SetIntakePivot(bool deployed) {
            double pivotSetpoint;
            if (deployed) {
                pivotSetpoint = IntakeConstants::kIntakeDown;
            } else {
                pivotSetpoint = IntakeConstants::kIntakeUp;
            }

            pivotMotor.GetClosedLoopController().SetSetpoint(pivotSetpoint, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot0, IntakeConstants::kFFPivot);

            
        }

        void SetIntakeSetpoint(double rot) override{
            pivotMotor.GetClosedLoopController().SetSetpoint(rot, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot0, IntakeConstants::kFFPivot);

        }

    private:
        ctre::phoenix6::hardware::TalonFX intakeMotor{IntakeConstants::kIntakeMotorPort};
        rev::spark::SparkMax pivotMotor{IntakeConstants::kIntakePivotPort, rev::spark::SparkLowLevel::MotorType::kBrushless};
        ctre::phoenix6::hardware::CANcoder pivotEncoder{IntakeConstants::kIntakeCANPort};
        ctre::phoenix6::controls::VoltageOut voltageRequest{0_V};
        
        void ConfigIntakeMotor() {
            ctre::phoenix6::configs::TalonFXConfiguration config{};
            config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            config.CurrentLimits.SupplyCurrentLimit = 45_A;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;

            intakeMotor.GetConfigurator().Apply(config);

        }

        void ConfigPivotMotor() {
            rev::spark::SparkMaxConfig config;
            config.closedLoop.P(0.1, rev::spark::kSlot0);
            config.closedLoop.I(0.0, rev::spark::kSlot0);
            config.closedLoop.D(0.0, rev::spark::kSlot0);

            config.closedLoop.P(0.02, rev::spark::kSlot1);
            config.closedLoop.I(0.0, rev::spark::kSlot1);
            config.closedLoop.D(0.0, rev::spark::kSlot1);

            config.SmartCurrentLimit(35, 50);
            config.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);
            config.OpenLoopRampRate(0.1);
            config.closedLoop.AllowedClosedLoopError(0.20);

            pivotMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
            pivotMotor.GetEncoder().SetPosition(0.0);
        }
};