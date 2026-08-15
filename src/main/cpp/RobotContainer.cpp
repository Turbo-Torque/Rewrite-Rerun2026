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
    ConfigureSetpointBindings();
    ConfigureNamedCommands();

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

void RobotContainer::ConfigureDefualts() {
    drivebaseSubsystem.SetDefaultCommand(CreateDriveCommand());
}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {
        drivebaseSubsystem.ZeroGyro();}));
    driveController.Y().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.GetPoseEstimator().SetVisionEnabled(!drivebaseSubsystem.GetPoseEstimator().GetVisionEnabled());}
    ));
}

void RobotContainer::ConfigureIntakeBindings() {
    
    driveController.A().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
    driveController.A().OnFalse(intakeSubsystem.AgitateCommand());
}

void RobotContainer::ConfigureFeedBindings() {

    operatorController.B().ToggleOnTrue(hopperSubsystem.RunHopperCommand().AlongWith(gateSubsystem.RunGateCommand()));
    
}

void RobotContainer::ConfigureShooterBindings(){
    operatorController.Y().ToggleOnTrue(shooterSubsystem.RunShooterCommand());
    operatorController.A().ToggleOnTrue(shooterSubsystem.TestShooter());
    operatorController.X().ToggleOnTrue(AimAndShootCommand());
    driveController.B().OnTrue(AimCommand());
}

void RobotContainer::ConfigureSetpointBindings() {
    operatorController.POVUp().ToggleOnTrue(shooterSubsystem.RunShooterCommand2(ShooterConstants::kShooterRPM1).AlongWith(shooterSubsystem.RunHoodCommand(ShooterConstants::kHoodAngle1)));
    operatorController.POVDown().ToggleOnTrue(shooterSubsystem.RunShooterCommand2(ShooterConstants::kShooterRPM2).AlongWith(shooterSubsystem.RunHoodCommand(ShooterConstants::kHoodAngle2)));
    operatorController.POVLeft().ToggleOnTrue(shooterSubsystem.RunShooterCommand2(ShooterConstants::kShooterRPM3).AlongWith(shooterSubsystem.RunHoodCommand(ShooterConstants::kHoodAngle3)));
    operatorController.POVRight().ToggleOnTrue(shooterSubsystem.RunShooterCommand2(ShooterConstants::kShooterRPM4).AlongWith(shooterSubsystem.RunHoodCommand(ShooterConstants::kHoodAngle4)));

}

void RobotContainer::ConfigureNamedCommands() {
      pathplanner::NamedCommands::registerCommand("Intake", intakeSubsystem.PivotAndRunIntakeCommand());
      pathplanner::NamedCommands::registerCommand("Feed", RunFeedCommand());
    pathplanner::NamedCommands::registerCommand("Shoot", shooterSubsystem.RunShooterCommand());

}


frc2::Command* RobotContainer::GetAutonomousCommand() {

    return autoChooser.GetSelected();

}