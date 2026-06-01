#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {

    ConfigureBindings();

    m_drivebaseSubsystem.SetDefaultCommand(
            frc2::cmd::Run(
            [this] {

                m_drivebaseSubsystem.Drive(

                    frc::ApplyDeadband(
                        -driveController.GetLeftY(),
                        0.05),

                    frc::ApplyDeadband(
                        -driveController.GetLeftX(),
                        0.05),

                    frc::ApplyDeadband(
                        -driveController.GetRightX(),
                        0.05)

                );
            },
            {&m_drivebaseSubsystem}
        )
    );
}

void RobotContainer::ConfigureBindings() {

    // Add controller bindings here

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

    return frc2::cmd::None();

}