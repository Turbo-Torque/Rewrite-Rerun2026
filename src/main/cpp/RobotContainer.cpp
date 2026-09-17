#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <units/math.h>
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

}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.ZeroGyro();}));
    driveController.Y().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.GetPoseEstimator().SetVisionEnabled(!drivebaseSubsystem.GetPoseEstimator().GetVisionEnabled());}));
    driveController.RightTrigger().ToggleOnTrue(drivebaseSubsystem.GetAngletoHubCommand());
    frc2::Trigger([this] {return drivebaseSubsystem.AtHeadingSetpoint();}).OnTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));

}

void RobotContainer::ConfigureIntakeBindings() {
    
    driveController.LeftTrigger().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
    operatorController.X().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
    driveController.LeftBumper().ToggleOnTrue(intakeSubsystem.SupplyPivotVoltsCommand());
    operatorController.RightBumper().WhileTrue(intakeSubsystem.AgitateCommand());
    operatorController.LeftBumper().WhileTrue(intakeSubsystem.PivotAndRunOuttakeCommand());
    operatorController.A().WhileTrue(AgitateSequenceCommand());
    frc2::Trigger([this] {return intakeSubsystem.IntakeNeedHopper();}).WhileTrue(ControllerRumble(driveController));
}

void RobotContainer::ConfigureFeedBindings() {    }

void RobotContainer::ConfigureShooterBindings(){
    operatorController.RightTrigger().WhileTrue(shooterSubsystem.RunShooterCommand().AlongWith(RunFeedCommand()).AlongWith(ControllerRumbleSShoot(operatorController)));
    operatorController.LeftTrigger().WhileTrue(shooterSubsystem.RunShooterCommand2().AlongWith(RunFeedCommand()).AlongWith(ControllerRumbleSShoot(operatorController)));
    // operatorController.B().ToggleOnTrue(shooterSubsystem.Laser(drivebaseSubsystem.GetPose().X()).AlongWith(RunFeedCommand()).AlongWith(ControllerRumbleWShoot(operatorController)));
    operatorController.B().ToggleOnTrue(shooterSubsystem.Laser([this] {
        auto muzzle = ShotSolve::GetMuzzlePosition(drivebaseSubsystem.GetPose());
        auto target = ShotSolve::GetTargetPosition();
        return units::math::abs(target.X() - muzzle.X());
    }).AlongWith(RunFeedCommand()).AlongWith(ControllerRumbleWShoot(operatorController)));

    // operatorController.A().ToggleOnTrue(shooterSubsystem.TestShooter().AlongWith(RunFeedCommand()));
    // operatorController.X().ToggleOnTrue(AimAndShootCommand());
}

void RobotContainer::ConfigureSetpointBindings() {
    // operatorController.POVUp().OnTrue(shooterSubsystem.SetShooterState(double (ShooterConstants::kShooterRPM3), ShooterConstants::kHoodAngle3).AlongWith(frc2::cmd::RunOnce([this] { drivebaseSubsystem.SelectCenterSetpoint();})));
    // operatorController.POVLeft().OnTrue(shooterSubsystem.SetShooterState(double (ShooterConstants::kShooterRPM1), ShooterConstants::kHoodAngle1).AlongWith(frc2::cmd::RunOnce([this] { drivebaseSubsystem.SelectLeftSetpoint();})));
    // operatorController.POVRight().OnTrue(shooterSubsystem.SetShooterState(double (ShooterConstants::kShooterRPM1), ShooterConstants::kHoodAngle1).AlongWith(frc2::cmd::RunOnce([this] { drivebaseSubsystem.SelectRightSetpoint();})));
    // operatorController.RightTrigger().ToggleOnTrue(shooterSubsystem.RunShooterCommand3().AlongWith(RunFeedCommand()));
    // driveController.RightTrigger().ToggleOnTrue(AlignToHub());
    // frc2::Trigger([this] {return drivebaseSubsystem.AtPoseSetPoint();}).OnTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));
}

void RobotContainer::ConfigureNamedCommands() {
    pathplanner::NamedCommands::registerCommand("Intake", intakeSubsystem.PivotAndRunIntakeCommand());
    pathplanner::NamedCommands::registerCommand("Agitate", intakeSubsystem.AgitateCommand());
    pathplanner::NamedCommands::registerCommand("Feed", RunFeedCommand());
    pathplanner::NamedCommands::registerCommand("Shoot", shooterSubsystem.RunShooterCommand());
    pathplanner::NamedCommands::registerCommand("Shoot 2", shooterSubsystem.RunShooterCommand2());
    pathplanner::NamedCommands::registerCommand("Align", drivebaseSubsystem.GetAngletoHubCommand());
    pathplanner::NamedCommands::registerCommand("Gyro", frc2::cmd::RunOnce([this] { drivebaseSubsystem.ZeroGyro(); }));
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}