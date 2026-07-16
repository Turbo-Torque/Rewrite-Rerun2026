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

  frc2::CommandPtr GetAutonomousCommand();

 private:
  DrivebaseSubsystem drivebaseSubsystem;
  IntakeSubsystem intakeSubsystem;
  HopperSubsystem hopperSubsystem;
  GateSubsystem gateSubsystem;
  ShooterSubsystem shooterSubsystem;

  frc2::CommandXboxController driveController{OperatorConstants::kDriveControllerPort};
  frc2::CommandXboxController operatorController{OperatorConstants::kOperatorControllerPort};

  frc2::CommandPtr CreateDriveCommand() {
    return drivebaseSubsystem.DriveCommand([this]() {return driveController.GetLeftX();}, [this]() {return driveController.GetLeftY();}, [this]() {return driveController.GetRightX();});
  }

   frc2::CommandPtr RunFeedCommand() {
    return frc2::cmd::WaitUntil([this] {
          return shooterSubsystem.IsNearState();
      })
      .AndThen(
          gateSubsystem.RunGateCommand()
              .AlongWith(hopperSubsystem.RunHopperCommand()));
    }

  void ConfigureBindings();
  void ConfigureDefualts();
  void ConfigureShooterBindings();
  void ConfigureIntakeBindings();
  void ConfigureFeedBindings();

};
