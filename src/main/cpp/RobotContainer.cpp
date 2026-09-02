#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

#include "frc/GenericHID.h"
#include "frc/Timer.h"
#include "subsystems/DrivebaseSubsystem.h"
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
#include "units/angular_velocity.h"


RobotContainer::RobotContainer()
    : drivebaseSubsystem(),
    intakeSubsystem(std::make_unique<IntakeRealIO>()),
    hopperSubsystem(std::make_unique<HopperRealIO>()),
    gateSubsystem(std::make_unique<GateRealIO>()),
    shooterSubsystem(std::make_unique<ShooterRealIO>()),
    hubSubsystem()

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
    driveController.B().ToggleOnTrue(drivebaseSubsystem.GetAngletoHubCommand());
    // driveController.B().ToggleOnTrue(AlignToHub().WithDeadline(frc2::cmd::Wait(2_s)));

}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.ZeroGyro();}));
    driveController.Y().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.GetPoseEstimator().SetVisionEnabled(!drivebaseSubsystem.GetPoseEstimator().GetVisionEnabled());}));
}

void RobotContainer::ConfigureIntakeBindings() {
    
    driveController.A().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
    operatorController.RightBumper().ToggleOnTrue(intakeSubsystem.AgitateCommand());
    frc2::Trigger([this] {
        return intakeSubsystem.IntakeNeedHopper();
    })
    .WhileTrue(hopperSubsystem.RunHopperCommand())
    .OnTrue(
        frc2::cmd::RunOnce([this] {
            driveController.SetRumble(
                frc::GenericHID::RumbleType::kBothRumble,
                1.0
            );
        })
    )
    .OnFalse(
        frc2::cmd::RunOnce([this] {
            driveController.SetRumble(
                frc::GenericHID::RumbleType::kBothRumble,
                0.0
            );
        })
    );
    operatorController.LeftBumper().ToggleOnTrue(intakeSubsystem.PivotAndRunOuttakeCommand().AlongWith(hopperSubsystem.Outtake()));
}

void RobotContainer::ConfigureFeedBindings() {

    // operatorController.B().ToggleOnTrue(hopperSubsystem.RunHopperCommand().AlongWith(gateSubsystem.RunGateCommand()));
    
}

void RobotContainer::ConfigureShooterBindings(){
    operatorController.Y().ToggleOnTrue(shooterSubsystem.RunShooterCommand().AlongWith(RunFeedCommand()));
    operatorController.A().ToggleOnTrue(shooterSubsystem.TestShooter());
    operatorController.X().ToggleOnTrue(AimAndShootCommand());
    operatorController.B().ToggleOnTrue(shooterSubsystem.Laser().AlongWith(RunFeedCommand()));
}

void RobotContainer::ConfigureSetpointBindings() {
    operatorController.POVUp().OnTrue(shooterSubsystem.SetShooterState(double (ShooterConstants::kShooterRPM1), ShooterConstants::kHoodAngle1).AlongWith(frc2::cmd::RunOnce([this] { drivebaseSubsystem.SelectCenterSetpoint();})));
    operatorController.POVLeft().OnTrue(shooterSubsystem.SetShooterState(double (ShooterConstants::kShooterRPM3), ShooterConstants::kHoodAngle3).AlongWith(frc2::cmd::RunOnce([this] { drivebaseSubsystem.SelectLeftSetpoint();})));
    operatorController.POVRight().OnTrue(shooterSubsystem.SetShooterState(double (ShooterConstants::kShooterRPM4), ShooterConstants::kHoodAngle4).AlongWith(frc2::cmd::RunOnce([this] { drivebaseSubsystem.SelectRightSetpoint();})));
    operatorController.RightTrigger().ToggleOnTrue(shooterSubsystem.RunShooterCommand3().AlongWith(RunFeedCommand()));
    driveController.RightTrigger().ToggleOnTrue(drivebaseSubsystem.GetPoseToSetpoint());
}

void RobotContainer::ConfigureNamedCommands() {
    pathplanner::NamedCommands::registerCommand("Intake", intakeSubsystem.PivotAndRunIntakeCommand());
    pathplanner::NamedCommands::registerCommand("Agitate", intakeSubsystem.AgitateCommand());
    pathplanner::NamedCommands::registerCommand("Feed", RunFeedCommand());
    pathplanner::NamedCommands::registerCommand("Shoot", shooterSubsystem.RunShooterCommand());
    pathplanner::NamedCommands::registerCommand("Align", drivebaseSubsystem.GetAngletoHubCommand());

}


frc2::Command* RobotContainer::GetAutonomousCommand() {

    return autoChooser.GetSelected();

}