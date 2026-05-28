#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {

  RobotContainer::RobotContainer() {

    ConfigureBindings();

    driveBaseSubsystem.SetDefaultCommand(

        frc2::cmd::Run(

            [this] {

                driveBaseSubsystem.Drive(

                    frc::ApplyDeadband(
                        -driveController.GetLeftY(),
                        0.05
                    ),

                    frc::ApplyDeadband(
                        -driveController.GetLeftX(),
                        0.05
                    ),

                    frc::ApplyDeadband(
                        -driveController.GetRightX(),
                        0.05
                    )

                );

            },

            {&driveBaseSubsystem}

        )

    );
}


void RobotContainer::ConfigureBindings() {

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::None();
}
