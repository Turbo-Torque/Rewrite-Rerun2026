#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule
    (int driveMotorID, int turnMotorID, int encoderID)
    : driveMotor(driveMotorID),
    turnMotor(turnMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    turnEncoder(encoderID) {
        turnPIDController.EnableContinuousInput(-180.0, 180.0);
    }

frc::SwerveModuleState SwerveModule::GetState() {
    return frc::SwerveModuleState{};
}
void SwerveModule::SetDesiredState(const frc::SwerveModuleState & state) {
        double driveSpeed = state.speed.value();
        double desiredAngle = state.angle.Degrees().value();
        double currentAngle = turnEncoder.GetAbsolutePosition().GetValue().value() * 360;

        double turnOutput = turnPIDController.Calculate(currentAngle, desiredAngle);

        driveMotor.Set(driveSpeed);
        turnMotor.Set(turnOutput);

    }
