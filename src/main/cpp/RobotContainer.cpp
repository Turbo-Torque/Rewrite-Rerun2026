#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <units/math.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

#include "frc/DriverStation.h"
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

    // ADDED: seed these ONCE — PutNumber every cycle would overwrite whatever you type into them in
    // Elastic. Periodic reads use GetNumber/GetBoolean, which reads back whatever value is currently
    // there (yours, if you changed it) rather than resetting it.
    frc::SmartDashboard::PutBoolean("Use Fake Pose", false);
    frc::SmartDashboard::PutNumber("Fake X", 0.0);
    frc::SmartDashboard::PutNumber("Fake Y", 0.0);
    // ADDED: live-tunable flywheel efficiency and obstacle offset — edit these in Elastic while
    // testing; ShotSolve::EvaluateCandidates() reads them fresh every cycle.
    frc::SmartDashboard::PutNumber("Flywheel Efficiency", 0.52);
    frc::SmartDashboard::PutNumber("Obstacle Distance From Target", ShooterConstants::kObstacleDistanceFromTarget.value());
}

void RobotContainer::ConfigureDefualts() {
    drivebaseSubsystem.SetDefaultCommand(CreateDriveCommand());

}
void RobotContainer::ConfigureBindings() {

    driveController.Start().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.ZeroGyro();}));
    driveController.Y().OnTrue(frc2::cmd::RunOnce([this] {drivebaseSubsystem.GetPoseEstimator().SetVisionEnabled(!drivebaseSubsystem.GetPoseEstimator().GetVisionEnabled());}));
    // REMOVED (by request): align-to-hub button binding. GetAngletoHubCommand() is still used by
    // the "Align" named command for autos (ConfigureNamedCommands) — only this teleop trigger is gone.
    driveController.RightTrigger().ToggleOnTrue(drivebaseSubsystem.GetAngletoHubCommand().WithDeadline(frc2::cmd::Wait(2_s)));
    // frc2::Trigger([this] {return drivebaseSubsystem.AtHeadingSetpoint();}).OnTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));

}

void RobotContainer::ConfigureIntakeBindings() {
    
    driveController.LeftTrigger().ToggleOnTrue(intakeSubsystem.PivotAndRunIntakeCommand());
    operatorController.X().ToggleOnTrue(intakeSubsystem.PivotAndRunOuttakeCommand().AlongWith(hopperSubsystem.Outtake()).AlongWith(gateSubsystem.RunOutake()));
    // FIXED: was calling ResetIntake() (a getter, return value discarded) instead of SetResetIntake()
    // (which actually sets resetIntake=true for the duration and back to false when this ends)
    driveController.LeftBumper().ToggleOnTrue(intakeSubsystem.SupplyPivotVoltsCommand().AlongWith(intakeSubsystem.SetResetIntake()));
    operatorController.RightBumper().WhileTrue(intakeSubsystem.AgitateCommand());
    // driveController.A().ToggleOnTrue(intakeSubsystem.SetIntakeToUp());
    operatorController.LeftBumper().WhileTrue(intakeSubsystem.PivotAndRunIntakeCommand());
    // operatorController.A().WhileTrue(AgitateSequenceCommand());
    // Teleop-only: these are sensor-driven (not button-driven), so without the mode check they'd
    // also fire during autonomous whenever the auto's own intake step hits a roller stall.
    frc2::Trigger([this] {return frc::DriverStation::IsTeleopEnabled() && intakeSubsystem.IntakeNeedHopper();}).WhileTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));
}

void RobotContainer::ConfigureFeedBindings() {    }

void RobotContainer::ConfigureShooterBindings(){
    // ADDED: the main shots require our alliance's hub to currently be active; Laser is exempt by
    // request. .Until(...) is checked every cycle (not just at button-press), so it cuts off
    // mid-shot too if the hub goes inactive while already running, not only at the next press.
    auto weCantShoot = [this] {
        return !hubSubsystem.IsAllianceActive(frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kRed));
    };
    // operatorController.RightTrigger().WhileTrue((shooterSubsystem.RunShooterCommand().AlongWith(RunFeedCommand())).Until(weCantShoot));
    // operatorController.LeftTrigger().WhileTrue((shooterSubsystem.RunShooterCommand2().AlongWith(RunFeedCommand())).Until(weCantShoot));
    operatorController.RightTrigger().WhileTrue((shooterSubsystem.RunShooterCommand().AlongWith(RunFeedCommand())));
    operatorController.LeftTrigger().WhileTrue((shooterSubsystem.RunShooterCommand2().AlongWith(RunFeedCommand())));
    // Teleop-only: without this the auto's Shoot step spinning up the shooter makes IsNearState()
    // go true and buzzes both controllers, even though nobody's holding them.
    frc2::Trigger([this] {return frc::DriverStation::IsTeleopEnabled() && shooterSubsystem.IsNearState();}).WhileTrue(ControllerRumble(driveController).AlongWith(ControllerRumble(operatorController)));
    operatorController.B().WhileTrue(shooterSubsystem.Laser([this] {
        auto muzzle = ShotSolve::GetMuzzlePosition(drivebaseSubsystem.GetPose());
        auto target = ShotSolve::GetTargetPosition();
        return units::math::abs(target.X() - muzzle.X());
    }).AlongWith(RunFeedCommand()).AlongWith(ControllerRumble(driveController)).AlongWith(ControllerRumble(operatorController)));

    // operatorController.A().ToggleOnTrue(shooterSubsystem.TestShooter().AlongWith(RunFeedCommand()));
    driveController.A().ToggleOnTrue(AimAndShootCommand().AlongWith(RunFeedCommand()));
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
    pathplanner::NamedCommands::registerCommand("Intake D", intakeSubsystem.PivotVolts());
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser.GetSelected();
}