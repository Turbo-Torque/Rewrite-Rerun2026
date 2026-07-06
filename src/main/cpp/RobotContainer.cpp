#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer():
    drivebaseSubsystem()
{

    ConfigureBindings();
    ConfigureDefualts();
}

void RobotContainer::ConfigureDefualts() {
    drivebaseSubsystem.SetDefaultCommand(CreateDriveCommand());
}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {
        drivebaseSubsystem.ZeroGyro();
    } ));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

    return frc2::cmd::None();

}