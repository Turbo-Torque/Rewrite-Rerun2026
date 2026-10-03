// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <cmath>

#include "Constants.h"
#include "frc/Timer.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/HopperSubsystem.h"
#include "subsystems/GateSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include <frc2/command/Commands.h>
#include "subsystems/ShotSolve.h"
#include "subsystems/HubSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"


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

  DrivebaseSubsystem& GetDrivebaseSubsystem() { return drivebaseSubsystem; }

  frc2::Command* GetAutonomousCommand();

  // ADDED: runs ShotSolve and publishes the result to Elastic every call, independent of any
  // button — meant to be called unconditionally from Robot::RobotPeriodic() so the dashboard
  // keeps showing "what the shot would be right now" continuously as the robot moves, without
  // needing AimAndShootCommand() to be actively toggled on. Does NOT command the shooter/hood —
  // actually shooting still requires pressing the button, this is display-only.
  void UpdateShotSolveDisplay() { ComputeBestShot(); }

 private:
  DrivebaseSubsystem drivebaseSubsystem;
  IntakeSubsystem intakeSubsystem;
  HopperSubsystem hopperSubsystem;
  GateSubsystem gateSubsystem;
  ShooterSubsystem shooterSubsystem;
  HubSubsystem hubSubsystem;

  frc2::CommandXboxController driveController{OperatorConstants::kDriveControllerPort};
  frc2::CommandXboxController operatorController{OperatorConstants::kOperatorControllerPort};

  frc2::CommandPtr CreateDriveCommand() {
    return drivebaseSubsystem.DriveCommand([this]() {return -(driveController.GetLeftY());}, [this]() {return -(driveController.GetLeftX());}, [this]() {return driveController.GetRightX();});
  }

    frc2::CommandPtr CreateDrive2Command() {

        return drivebaseSubsystem.Drive2Command([this]() {return -(driveController.GetLeftY());}, [this]() {return -(driveController.GetLeftX());}, [this]() {
            double x = driveController.GetRightY();
            double y = driveController.GetRightX();
            return std::atan2(-y, -x) * 180.0 / M_PI;
        });
    }


  frc2::CommandPtr CreateRotateAndDriveCommand() {
    return drivebaseSubsystem.RotateToHubAndDriveCommand([this] {return -(driveController.GetLeftY());}, [this] {return -(driveController.GetLeftX());}, drivebaseSubsystem.GetBestFutureTargetAngle());
  }

  frc2::CommandPtr RunFeedCommand() {
  return frc2::cmd::WaitUntil([this] {
        return shooterSubsystem.IsNearState();
    }).WithTimeout(2_s)
    .AndThen(
        hopperSubsystem.RunHopperCommand().AlongWith(gateSubsystem.RunGateCommand())
    );
  }

  frc2::CommandPtr IntakeNeedHopper() {
      return frc2::cmd::WaitUntil([this] {
          return intakeSubsystem.IntakeNeedHopper();
      })
      .AndThen(
          hopperSubsystem.RunHopperCommand()
              .WithTimeout(2_s)
      )
      .Repeatedly();
  }

    frc2::CommandPtr AgitateSequenceCommand() {
        return intakeSubsystem.PivotAndRunIntakeCommand()
            .WithTimeout(3_s)
            .AndThen(intakeSubsystem.AgitateCommand());
    }


frc2::CommandPtr AlignToHub() {
    return drivebaseSubsystem.GetPoseToSetpoint().WithDeadline(frc2::cmd::Wait(3_s))
        .AndThen(drivebaseSubsystem.GetAngletoHubCommand());
}



  // ADDED: shared by AimAndShootCommand() and UpdateShotSolveDisplay() so the continuous dashboard
  // display and the actual shoot command always compute the exact same thing, once, in one place.
  std::optional<ShotSolve::ShotEvaluation> ComputeBestShot() {

      frc::Pose2d currentPose = drivebaseSubsystem.GetPose();

      auto muzzle = ShotSolve::GetMuzzlePosition(currentPose);
      auto target = ShotSolve::GetTargetPosition();  // handles Red/Blue internally via DriverStation::GetAlliance()
      auto geometry = ShotSolve::ComputeGeometry(muzzle, target);

      auto candidates = ShotSolve::GenerateCandidates();
      auto evaluations = ShotSolve::EvaluateCandidates(candidates, geometry);
      auto validShots = ShotSolve::FilterValidShots(evaluations, geometry);

      ShotSolve::RankShots(validShots);
      auto shot = ShotSolve::SelectBestShot(validShots);

      frc::SmartDashboard::PutBoolean("ShotSolve Has Valid Shot", shot.has_value());
      if (shot) {

          double hoodSetpoint = shooterSubsystem.AngleToHoodSetpoint(shot->candidate.hoodAngle);
          frc::SmartDashboard::PutNumber("ShotSolve Best RPM", shot->candidate.rpm.value());
          frc::SmartDashboard::PutNumber("ShotSolve Best Hood Angle (true)", shot->candidate.hoodAngle.value());
          frc::SmartDashboard::PutNumber("ShotSolve Best Hood Setpoint (raw)", hoodSetpoint);
      }
      return shot;
  }

  frc2::CommandPtr AimAndShootCommand() {
      return frc2::cmd::Run([this] {
          auto shot = ComputeBestShot();

          if (shot) {
              double hoodSetpoint = shooterSubsystem.AngleToHoodSetpoint(shot->candidate.hoodAngle);
              shooterSubsystem.SetShooterRPM(shot->candidate.rpm);
              shooterSubsystem.SetHoodSetpoint(hoodSetpoint);
          }
    }, {&shooterSubsystem})
    .FinallyDo([this] {
        shooterSubsystem.CoastOut();
        shooterSubsystem.SetHoodSetpoint(ShooterConstants::kHoodDown);    });
  }

frc2::CommandPtr ControllerRumble(frc2::CommandXboxController& controller) {
    return frc2::cmd::Run([&controller] {
        controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
    })
    .FinallyDo([&controller] {
        controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    });
}

frc2::CommandPtr ControllerRumbleSShoot(frc2::CommandXboxController& controller) {
    return frc2::cmd::Run([&controller] {
        controller.SetRumble(frc::GenericHID::RumbleType::kRightRumble, .5);
    })
    .FinallyDo([&controller] {
        controller.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
    });
}

frc2::CommandPtr ControllerRumbleWShoot(frc2::CommandXboxController& controller) {
    return frc2::cmd::Run([&controller] {
        controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, .5);
    })
    .FinallyDo([&controller] {
        controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    });
}


  void ConfigureBindings();
  void ConfigureDefualts();
    void ConfigureNamedCommands();
  void ConfigureShooterBindings();
  void ConfigureIntakeBindings();
  void ConfigureFeedBindings();
  void ConfigureSetpointBindings();

  frc::SendableChooser<frc2::Command*> autoChooser;

};
