// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <string>

#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "ctre/phoenix6/CANBus.hpp"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>
#include <numbers>

#include "units/current.h"
#include "units/length.h"
#include "units/velocity.h"

inline constexpr double GEAR_RATIO = 6.75;
inline constexpr units::inch_t WHEEL_DIAMETER = 4_in;

namespace turbolib::motors {
class NeoKrakenModule final {
  std::string name;

  ctre::phoenix6::CANBus canBus;
  ctre::phoenix6::hardware::TalonFX driveMotor;
  rev::spark::SparkMax steerMotor;
  ctre::phoenix6::hardware::CANcoder encoderObject;

  double setpoint = 0.0;

  frc::SimpleMotorFeedforward<units::meters> ff;
  frc::PIDController driveController, steerController;

  constexpr static double kVelocityMultiplier =
      (1 / GEAR_RATIO / 60) * (units::meter_t{WHEEL_DIAMETER}.value() * std::numbers::pi);
  constexpr static double kPositionMultiplier =
      (1 / GEAR_RATIO) * (units::meter_t{WHEEL_DIAMETER}.value() * std::numbers::pi);
  constexpr static double kCanCoderMultiplier = 2 * std::numbers::pi;

  void ConfigDriveMotor(ctre::phoenix6::hardware::TalonFX &target);
  void ConfigSteerMotor(rev::spark::SparkMax &target);
  void SetupEncoder(ctre::phoenix6::hardware::CANcoder &encoder);
  void
  CurrentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration &config);

public:
  NeoKrakenModule(const std::string &name, int driveID, int steerID,
                  int encoderID, const std::string &can = "");
  ~NeoKrakenModule() = default;

  /// Configures the feedforward and PID values for the drive and steer motors.
  void ConfigPIDInternal();

  /// Sets the desired state of the module.
  void SetModuleState(frc::SwerveModuleState state);

  /// Returns the angle of the module in radians.
  double GetEncoderPosition();

  /// Returns the position of the module in meters.
  double GetPosition();

  /// Returns the state of the module, including velocity in meters per second
  /// and angle in radians.
  frc::SwerveModuleState GetModuleState();

  /// Returns the position of the module in meters and radians, respectively.
  frc::SwerveModulePosition GetModulePosition();

  /// Returns the velocity of the module in meters per second.
  units::meters_per_second_t GetVelocity();

  /// Returns the current draw of the drive motor in amperes.
  units::ampere_t GetCurrentDraw();

  ctre::phoenix6::hardware::TalonFX &GetDriveMotor();
};
} // namespace turbolib::motors
