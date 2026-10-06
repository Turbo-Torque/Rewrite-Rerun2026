#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <units/math.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

#include "frc/DriverStation.h"
#include "frc/GenericHID.h"
#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"
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


    frc::SmartDashboard::PutBoolean("Use Fake Pose", false);
    frc::SmartDashboard::PutNumber("Fake X", 0.0);
    frc::SmartDashboard::PutNumber("Fake Y", 0.0);

    // Flywheel Efficiency and Rotate Lead Time are locked in as constants now
    // (ShooterConstants::kFlywheelEfficiency / DriveConstants::kRotateLeadTime) — no longer seeded here.
    frc::SmartDashboard::PutNumber("Obstacle Distance From Target", ShooterConstants::kObstacleDistanceFromTarget.value());
}

void RobotContainer::ConfigureDefualts() {
    drivebaseSubsystem.SetDefaultCommand(CreateDriveCommand());
    // drivebaseSubsystem.SetDefaultCommand(CreateDrive2Command());
}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.ZeroGyro();}));
    driveController.Y().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.GetPoseEstimator().SetVisionEnabled(!drivebaseSubsystem.GetPoseEstimator().GetVisionEnabled());}));
    driveController.RightTrigger().ToggleOnTrue(CreateRotateAndDriveCommand());


}

void RobotContainer::ConfigureIntakeBindings() {
    
    driveController.LeftTrigger().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());

    driveController.LeftBumper().ToggleOnTrue(intakeSubsystem.SupplyPivotVoltsCommand().AlongWith(intakeSubsystem.SetResetIntake()));
    operatorController.RightBumper().WhileTrue(intakeSubsystem.AgitateCommand());
    operatorController.LeftBumper().WhileTrue(intakeSubsystem.PivotAndRunIntakeCommand());

    frc2::Trigger([this] {return frc::DriverStation::IsTeleopEnabled() && intakeSubsystem.IntakeNeedHopper();}).WhileTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));
}

void RobotContainer::ConfigureFeedBindings() {    }

void RobotContainer::ConfigureShooterBindings(){

    auto weCantShoot = [this] {
        return !hubSubsystem.IsAllianceActive(frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kRed));
    };

    // operatorController.RightTrigger().WhileTrue((shooterSubsystem.RunShooterCommand().AlongWith(RunFeedCommand())));
    operatorController.LeftTrigger().WhileTrue((shooterSubsystem.RunShooterCommand2().AlongWith(RunFeedCommand())));

    frc2::Trigger([this] {return frc::DriverStation::IsTeleopEnabled() && shooterSubsystem.IsNearState();}).WhileTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));
    operatorController.B().WhileTrue(shooterSubsystem.Laser([this] {
        auto muzzle = ShotSolve::GetMuzzlePosition(drivebaseSubsystem.GetPose());
        auto target = ShotSolve::GetTargetPosition();
        return units::math::abs(target.X() - muzzle.X());
    }).AlongWith(RunFeedCommand()).AlongWith(ControllerRumble(driveController)).AlongWith(ControllerRumble(operatorController)));

    operatorController.RightTrigger().ToggleOnTrue(AimAndShootCommand().AlongWith(RunFeedCommand()));
}

void RobotContainer::ConfigureSetpointBindings() {
    // frc2::Trigger([this] {return drivebaseSubsystem.AtPoseSetPoint();}).OnTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));
}

void RobotContainer::ConfigureNamedCommands() {
    pathplanner::NamedCommands::registerCommand("Intake", intakeSubsystem.PivotAndRunIntakeCommand());
    pathplanner::NamedCommands::registerCommand("Agitate", intakeSubsystem.AgitateCommand());
    pathplanner::NamedCommands::registerCommand("Feed", RunFeedCommand());
    pathplanner::NamedCommands::registerCommand("Shoot", shooterSubsystem.RunShooterCommand());
    pathplanner::NamedCommands::registerCommand("Shoot 2", shooterSubsystem.RunShooterCommand2());
    pathplanner::NamedCommands::registerCommand("Align", drivebaseSubsystem.GetAngletoHubCommand());
    pathplanner::NamedCommands::registerCommand("Auto Shoot", AimAndShootCommand());
    pathplanner::NamedCommands::registerCommand("Gyro", frc2::cmd::RunOnce([this] { drivebaseSubsystem.ZeroGyro(); }));
    pathplanner::NamedCommands::registerCommand("Intake D", intakeSubsystem.PivotVolts());
    pathplanner::NamedCommands::registerCommand("Auto Shoot",AimAndShootCommand().AlongWith(RunFeedCommand()));
    
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}

