#include "subsystems/SwerveModule.h"
#include "frc/DataLogManager.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/RelativeEncoder.h>
#include <cmath>


SwerveModule::SwerveModule
    (int driveMotorID, int turnMotorID, int encoderID)
    : driveMotor(driveMotorID),
    turnMotor(turnMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    turnEncoder(encoderID) {
        turnMotor.SetInverted(true);
        turnPIDController.EnableContinuousInput(-M_PI, M_PI);
    }
 
frc::SwerveModuleState SwerveModule::GetState() {
    double currentAngle = turnEncoder.GetAbsolutePosition().GetValue().value() * 2.0 * M_PI;
    return frc::SwerveModuleState{units::meters_per_second_t(GetCurrentSpeed()), frc::Rotation2d{units::radian_t(GetTurnEncoder())}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState & state) {

        double currentAngle = GetTurnEncoder();
        frc::SwerveModuleState optimizeState = state;
        optimizeState.Optimize(frc::Rotation2d{units::radian_t(currentAngle)});
        frc::Rotation2d error = optimizeState.angle - frc::Rotation2d{units::radian_t(currentAngle)};

        double driveSpeed = optimizeState.speed.value() * error.Cos();
        double desiredAngle = optimizeState.angle.Radians().value();
        double turnOutput = turnPIDController.Calculate(GetTurnEncoder(), optimizeState.angle.Radians().value()); // try - turnoutput to see if works.
        turnOutput = std::clamp(turnOutput, -1.0, 1.0);

        frc::SmartDashboard::PutNumber("current angle",currentAngle);
        frc::SmartDashboard::PutNumber(
            "Desired Angle",
            optimizeState.angle.Degrees().value());

        frc::SmartDashboard::PutNumber(
            "Drive Speed",
            optimizeState.speed.value());

        frc::SmartDashboard::PutNumber(
        "Current Angle",
        currentAngle);

        driveMotor.Set(driveSpeed);
        turnMotor.Set(turnOutput);
    }

void SwerveModule::SetDriveInverted(bool inverted) {
    turnMotor.SetInverted(inverted);
}

double SwerveModule::GetCurrentAngle() {
    return turnEncoder.GetAbsolutePosition().GetValue().value() * 2 * M_PI;
}


double SwerveModule::GetCurrentSpeed() {
    return driveMotor.Get();
}

double SwerveModule::GetTurnEncoder() {
    double angle = turnEncoder.GetAbsolutePosition().GetValue().value() * 2.0 * M_PI;
    angle = std::fmod(angle, 2.0 * M_PI);

    if (angle < 0) angle += 2.0 * M_PI;

    return angle;
}

