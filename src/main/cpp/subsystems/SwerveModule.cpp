#include "subsystems/SwerveModule.h"
#include "frc/DataLogManager.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule
    (int driveMotorID, int turnMotorID, int encoderID)
    : driveMotor(driveMotorID),
    turnMotor(turnMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    turnEncoder(encoderID) {
        turnPIDController.EnableContinuousInput(-180, 180);
        turnMotor.SetInverted(true);
    }
 
frc::SwerveModuleState SwerveModule::GetState() {
    return frc::SwerveModuleState{};


}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState & state) {
        
        double currentAngle = turnEncoder.GetAbsolutePosition().GetValue().value() * 360;
        frc::SwerveModuleState optimizeState = frc::SwerveModuleState::Optimize(state, frc::Rotation2d{units::degree_t(currentAngle)});
        double driveSpeed = optimizeState.speed.value();
        double desiredAngle = optimizeState.angle.Degrees().value();
        double turnOutput = turnPIDController.Calculate(currentAngle, desiredAngle);
        turnOutput = std::clamp(turnOutput, -1.0, 1.0);

        frc::SmartDashboard::PutNumber("desired angle",desiredAngle);
        frc::SmartDashboard::PutNumber("current angle",currentAngle);
        frc::SmartDashboard::PutNumber("angle", turnOutput);

        PublishInfo("current angle", currentAngle);
    

        driveMotor.Set(driveSpeed);
        turnMotor.Set(turnOutput);

    }




