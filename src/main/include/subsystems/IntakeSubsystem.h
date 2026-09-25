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
        // FIXED: was declared taking a bool the .cpp definition never had (signature mismatch, wouldn't link)
        frc2::CommandPtr SetResetIntake();
        bool ResetIntake();
        bool AtPivotSetpoint(double target, double tolerance = 2.0) const {
            return std::abs(inputs.position - target) <= tolerance;
        }

        frc2::CommandPtr PivotAndRunIntakeCommand();
        frc2::CommandPtr PivotAndRunOuttakeCommand();
        frc2::CommandPtr AgitateCommand();
        frc2::CommandPtr SupplyPivotVoltsCommand();
        frc2::CommandPtr PivotVolts();
        frc2::CommandPtr SetIntakeToUp();
        void Periodic() override;

    private:
        // ADDED: while resetIntake is true, wrapping a command in this makes it a no-op instead of
        // running — used so no other pivot command can start (and fight the mechanism) mid-recalibration.
        frc2::CommandPtr GuardedByReset(frc2::CommandPtr&& command);

        std::unique_ptr<IntakeIO> io;
        IntakeIOInputs inputs;

        bool intakeDeployed = false;
        bool resetIntake = false;
        double newIntakePivot = 0;
        double pivotOffset = 0.0;
};
