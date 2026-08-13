#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/IntakeSubsystem.h"
#include "abstractions/io/intake/IntakeIO.h"
#include "abstractions/io/intake/IntakeRealIO.h"

#include "subsystems/HopperSubsystem.h"
#include "abstractions/io/hopper/HopperIO.h"
#include "abstractions/io/hopper/HopperRealIO.h"

#include "subsystems/GateSubsystem.h"
#include "abstractions/io/gate/GateIO.h"
#include "abstractions/io/gate/GateRealIO.h"


#include "subsystems/ShooterSubsystem.h"
#include "abstractions/io/shooter/ShooterIO.h"
#include "abstractions/io/shooter/ShooterRealIO.h"

#include "turbolib/util/MakeIO.hpp"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>
#include "pathplanner/lib/auto/AutoBuilder.h"

#include "Constants.h"

RobotContainer::RobotContainer()
    : drivebaseSubsystem(),
    intakeSubsystem(std::make_unique<IntakeRealIO>()),
    hopperSubsystem(std::make_unique<HopperRealIO>()),
    gateSubsystem(std::make_unique<GateRealIO>()),
    shooterSubsystem(std::make_unique<ShooterRealIO>())

{
    ConfigureBindings();
    ConfigureDefualts();
    ConfigureIntakeBindings();
    ConfigureFeedBindings();
    ConfigureShooterBindings();
    ConfigureNamedCommands();

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
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
    driveController.A().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
}

void RobotContainer::ConfigureFeedBindings() {

    operatorController.B().ToggleOnTrue(hopperSubsystem.RunHopperCommand().AlongWith(gateSubsystem.RunGateCommand()));
    
}

void RobotContainer::ConfigureSetpointBindings() {
    operatorController.POVUp().(shooterSubsystem.SetHoodSetpoint(ShooterConstants::kHoodAngle1));
    operatorController.POVDown().ToggleOnTrue(shooterSubsystem.SetHoodSetpoint(ShooterConstants::kHoodAngle2));
    operatorController.POVLeft().ToggleOnTrue(shooterSubsystem.SetHoodSetpoint(ShooterConstants::kHoodAngle3));
    operatorController.POVRight().ToggleOnTrue(shooterSubsystem.SetHoodSetpoint(ShooterConstants::kHoodAngle4));
    operatorController.POVUp().ToggleOnTrue(shooterSubsystem.RunShooterCommand(ShooterConstants::kShooterRPM1));
    operatorController.POVDown().ToggleOnTrue(shooterSubsystem.RunShooterCommand(ShooterConstants::kShooterRPM2));
    operatorController.POVLeft().ToggleOnTrue(shooterSubsystem.RunShooterCommand(ShooterConstants::kShooterRPM3));
    operatorController.POVRight().ToggleOnTrue(shooterSubsystem.RunShooterCommand(ShooterConstants::kShooterRPM4));

}

void RobotContainer::ConfigureShooterBindings(){
    operatorController.Y().ToggleOnTrue(shooterSubsystem.RunShooterCommand(ShooterConstants::kShooterRPM));
    operatorController.X().ToggleOnTrue(AimAndShootCommand());
    operatorController.A().ToggleOnTrue(shooterSubsystem.RunLaseringCommand());
    driveController.B().OnTrue(AimCommand());
}

void RobotContainer::ApplyStartingPose() {
    drivebaseSubsystem.ApplyStartingPose();
}

void RobotContainer::ConfigureNamedCommands() {
      pathplanner::NamedCommands::registerCommand("Intake", intakeSubsystem.PivotAndRunIntakeCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

    return autoChooser.GetSelected();

}