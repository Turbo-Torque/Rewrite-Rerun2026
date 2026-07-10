#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include "subsystems/IntakeSubsystem.h"
#include "abstractions/io/intake/IntakeIO.h"
#include "abstractions/io/intake/IntakeRealIO.h"
#include "abstractions/io/intake/IntakeSimIO.h"
#include "subsystems/HopperSubsystem.h"
#include "abstractions/io/hopper/HopperIO.h"
#include "abstractions/io/hopper/HopperRealIO.h"
#include "abstractions/io/hopper/HopperSimIO.h"
#include "turbolib/util/MakeIO.hpp"


RobotContainer::RobotContainer()
    : drivebaseSubsystem(),
    intakeSubsystem(turbolib::utils::MakeIO<IntakeIO, IntakeRealIO, IntakeSimIO>()),
    hopperSubsystem(turbolib::utils::MakeIO<HopperIO, HopperRealIO, HopperSimIO>())

{
    ConfigureBindings();
    ConfigureDefualts();
    ConfigureIntakeBindings();
}

void RobotContainer::ConfigureDefualts() {
    drivebaseSubsystem.SetDefaultCommand(CreateDriveCommand());
}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {
        drivebaseSubsystem.ZeroGyro();
    } ));
}

void RobotContainer::ConfigureIntakeBindings() {
    operatorController.A().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
}



frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

    return frc2::cmd::None();

}