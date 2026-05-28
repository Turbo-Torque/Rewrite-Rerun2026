#include "subsystems/SwerveModule.h"

SwerveModule::swerveModule
    (int driveMotorID, int turnMotorID, int encoderID)
    : driveMotor(driveMotorID),
    turnMotor(turnMotorID, rev::spark::SparkLowLevel::motorType::kBrushless),
    turnEncoder(encoderID) {
        turningPID.EnableContinuousInput(-180.0, 180.0);
    }

frc::SwerveModuleState SwerveModule::GetState() {
    return frc::SwerveModuleState{};
}
void ServeModule::SetDesiredState(
    const frc::SwerveModuleState & state {
        double driveSpeed = state.speed.value();
        double desiredAngle = state.angle.Degrees().value();
        double currentAngle = turnEncoder.GetAbsolutePosition().getValue()*360;

        double turnOutput = turningPID.Calculate(currentAngle, desiredAngle);

        driveMotor.Set(driveSpeed);
        turnMotor.Set(turnOutput);

    }
)