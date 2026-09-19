#pragma once
#include "Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/intake/IntakeIO.h"
#include "opencv2/core/cvstd.hpp"


class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(std::unique_ptr<IntakeIO> io);

        void SetIntakeVoltage(units::volt_t voltage) { io -> SetIntakeVolts(voltage);}
        void SetAgitateVolts(units::volt_t volts) { io -> SetAgitateVolts(volts);}
        void SetPivotVolts(units::volt_t volts) { io -> SetPivotVolts(volts);}
        void SetIntakeSetpoint(double rot) {io -> SetIntakeSetpoint(rot);}
        void Agitate(double rot) {io -> Agitate(rot);}


        bool IntakeNeedHopper();
        bool AgitateRollers();
        bool AtPivotSetpoint(double target, double tolerance = 2.0) const {
            return std::abs(inputs.position - target) <= tolerance;
        }

        frc2::CommandPtr PivotAndRunIntakeCommand();
        frc2::CommandPtr PivotAndRunOuttakeCommand();
        frc2::CommandPtr AgitateCommand();
        frc2::CommandPtr SupplyPivotVoltsCommand();
        frc2::CommandPtr PivotVolts();
        void Periodic() override;

    private:
        std::unique_ptr<IntakeIO> io;
        IntakeIOInputs inputs;

        bool intakeDeployed = false;
        double newIntakePivot = 0;
        double pivotOffset = 0.0;
};
