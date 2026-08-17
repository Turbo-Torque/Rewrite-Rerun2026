#pragma once
#include "frc2/command/SubsystemBase.h"
#include "abstractions/io/intake/IntakeIO.h"


class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(std::unique_ptr<IntakeIO> io);

        void SetIntakeVoltage(units::volt_t voltage) { io -> SetIntakeVolts(voltage);}
        void SetIntakeSetpoint(double rot) {io -> SetIntakeSetpoint(rot);}

        frc2::CommandPtr PivotAndRunIntakeCommand();
        frc2::CommandPtr AgitateCommand();
        void Periodic() override;

    private:
        std::unique_ptr<IntakeIO> io;
        IntakeIOInputs inputs;

        bool intakeDeployed = false;
};
