// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/motors/NeoKrakenModule.hpp"

#include <cmath>
#include <string>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>

#include <rev/ConfigureTypes.h>
#include <rev/SparkBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkBaseConfig.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <numbers>

#include "ctre/phoenix6/TalonFX.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/voltage.h"


namespace {
constexpr double kDriveP = 0.01;
constexpr double kSteerP = 0.3;
constexpr units::volt_t kKs = 0.2_V;
constexpr auto kKv = 2.0_V / 1_mps;
}  // namespace

using namespace turbolib::motors;

NeoKrakenModule::NeoKrakenModule(const std::string& name, const int driveID, const int steerID, const int encoderID,
                                 const std::string& can)
    : name(name),
      canBus(can),
      driveMotor(driveID, canBus),
      steerMotor(steerID, rev::spark::SparkLowLevel::MotorType::kBrushless),
      encoderObject(encoderID, canBus),
      ff(0_V, 0_V / 1_mps),
      driveController(0.0, 0.0, 0.0),
      steerController(0.0, 0.0, 0.0) {
  SetupEncoder(encoderObject);
  ConfigDriveMotor(driveMotor);
  ConfigSteerMotor(steerMotor);
  ConfigPIDInternal();
}

void NeoKrakenModule::SetupEncoder(ctre::phoenix6::hardware::CANcoder& encoder) {
  ctre::phoenix6::configs::CANcoderConfigurator& configPls = encoder.GetConfigurator();
  ctre::phoenix6::configs::CANcoderConfiguration config{};
  configPls.Refresh(config);

  config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::turn_t{0.5};

  configPls.Apply(config);
}

void NeoKrakenModule::ConfigPIDInternal() {
  this->ff = frc::SimpleMotorFeedforward<units::meters>{kKs, kKv};

  this->driveController = frc::PIDController(kDriveP, 0.0, 0.0);
  this->steerController = frc::PIDController(kSteerP, 0.0, 0.0);

  this->steerController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void NeoKrakenModule::ConfigDriveMotor(ctre::phoenix6::hardware::TalonFX& target) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};

  target.GetConfigurator().Refresh(config);

  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  CurrentLimitsDrive(config);

  target.GetConfigurator().Apply(config);
}

void NeoKrakenModule::ConfigSteerMotor(rev::spark::SparkMax& target) {
  rev::spark::SparkBaseConfig config;
  config.Inverted(false);
  config.VoltageCompensation(12);
  config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  config.SmartCurrentLimit(80);
  target.Configure(config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);
}

// Configure Kraken Motor's CurrentLimits
void NeoKrakenModule::CurrentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration& config) {
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimitEnable = true;

  config.CurrentLimits.SupplyCurrentLimit = 50_A;
  config.CurrentLimits.StatorCurrentLimit = 70_A;
}

void NeoKrakenModule::SetModuleState(frc::SwerveModuleState state) {
  const double currentMeasurement = GetEncoderPosition();
  const frc::Rotation2d currentAngle{units::radian_t{currentMeasurement}};
  state.Optimize(currentAngle);

  const units::meters_per_second_t compensatedSpeed = state.speed * (state.angle - currentAngle).Cos();

  const units::radian_t angle = state.angle.Radians();
  setpoint = angle.value();

  units::volt_t output = ff.Calculate(compensatedSpeed) +
                         units::volt_t{driveController.Calculate(GetVelocity().value(), compensatedSpeed.value())};
  const double steerPercent = steerController.Calculate(currentMeasurement, angle.value());

  driveMotor.SetControl(ctre::phoenix6::controls::VoltageOut{output});
  steerMotor.Set(-steerPercent);
}

double NeoKrakenModule::GetEncoderPosition() {
  const ctre::phoenix6::StatusSignal<units::turn_t> angle = encoderObject.GetAbsolutePosition();

  const double rawValue = angle.GetValueAsDouble();
  const double rawRadians = rawValue * kCanCoderMultiplier;

  return rawRadians;
}

double NeoKrakenModule::GetPosition() {
  const ctre::phoenix6::StatusSignal<units::turn_t> position = driveMotor.GetPosition();
  return position.GetValue().value() * kPositionMultiplier;
}

frc::SwerveModuleState NeoKrakenModule::GetModuleState() {
  return {GetVelocity(), frc::Rotation2d{units::radian_t{GetEncoderPosition()}}};
}

frc::SwerveModulePosition NeoKrakenModule::GetModulePosition() {
  return {units::meter_t{GetPosition()}, units::radian_t{std::numbers::pi + GetEncoderPosition()}};
}

units::meters_per_second_t NeoKrakenModule::GetVelocity() {
  const ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocity = driveMotor.GetVelocity();
  return units::meters_per_second_t{velocity.GetValue().value() * kVelocityMultiplier};
}

units::ampere_t NeoKrakenModule::GetCurrentDraw() {
  return driveMotor.GetTorqueCurrent().GetValue();
}

ctre::phoenix6::hardware::TalonFX& NeoKrakenModule::GetDriveMotor() {
  return driveMotor;
}