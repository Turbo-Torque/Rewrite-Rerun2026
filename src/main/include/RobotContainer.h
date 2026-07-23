// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/HopperSubsystem.h"
#include "subsystems/GateSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include <frc2/command/Commands.h>
#include "subsystems/ShotSolve.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  void ApplyStartingPose();

 private:
  DrivebaseSubsystem drivebaseSubsystem;
  IntakeSubsystem intakeSubsystem;
  HopperSubsystem hopperSubsystem;
  GateSubsystem gateSubsystem;
  ShooterSubsystem shooterSubsystem;

  frc2::CommandXboxController driveController{OperatorConstants::kDriveControllerPort};
  frc2::CommandXboxController operatorController{OperatorConstants::kOperatorControllerPort};


  frc2::CommandPtr CreateDriveCommand() {
    return drivebaseSubsystem.DriveCommand([this]() {return driveController.GetLeftY();}, [this]() {return driveController.GetLeftX();}, [this]() {return driveController.GetRightX();});
  }

  frc2::CommandPtr RunFeedCommand() {
  return frc2::cmd::WaitUntil([this] {
        return shooterSubsystem.IsNearState() && drivebaseSubsystem.AtHeadingSetpoint();
    })
    .AndThen(
        hopperSubsystem.RunHopperCommand());
  }

  frc2::CommandPtr AimCommand() {
      return frc2::cmd::Run([this] {
          auto muzzle = ShotSolve::GetMuzzlePosition(drivebaseSubsystem.GetPose());
          auto target = ShotSolve::GetTargetPosition();
          auto geometry = ShotSolve::ComputeGeometry(muzzle, target);
          drivebaseSubsystem.AimAtHeading(geometry.bearing);
      }, {&drivebaseSubsystem})
      .Until([this] { return drivebaseSubsystem.AtHeadingSetpoint(); });
  }

  frc2::CommandPtr AimAndShootCommand() {
      return frc2::cmd::Run([this] {
          auto muzzle = ShotSolve::GetMuzzlePosition(drivebaseSubsystem.GetPose());
          auto target = ShotSolve::GetTargetPosition();
          auto geometry = ShotSolve::ComputeGeometry(muzzle, target);

          auto candidates = ShotSolve::GenerateCandidates();
          auto evaluations = ShotSolve::EvaluateCandidates(candidates, geometry);
          auto validShots = ShotSolve::FilterValidShots(evaluations, geometry);

          ShotSolve::RankShots(validShots);
          auto shot = ShotSolve::SelectBestShot(validShots);

          drivebaseSubsystem.AimAtHeading(geometry.bearing);

          if (shot) {
              shooterSubsystem.SetShooterRPM(shot->candidate.rpm);
              shooterSubsystem.SetHoodSetpoint(shot->candidate.hoodAngle.value());
          }
    }, {&drivebaseSubsystem, &shooterSubsystem})
    .FinallyDo([this] {
        shooterSubsystem.SetShooterRPM(0_rpm);
    });
  }

  void ConfigureBindings();
  void ConfigureDefualts();
  void ConfigureShooterBindings();
  void ConfigureIntakeBindings();
  void ConfigureFeedBindings();

  frc::SendableChooser<frc2::Command*> autoChooser;

};
