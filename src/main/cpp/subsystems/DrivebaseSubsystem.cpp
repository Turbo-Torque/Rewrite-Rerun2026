#include "subsystems/DrivebaseSubsystem.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

DrivebaseSubsystem::DrivebaseSubsystem() {};

void DrivebaseSubsystem::Drive(double xSpeed, double ySpeed, double rotation) {
    frc::ChassisSpeeds speeds{    
    units::meters_per_second_t{xSpeed * 0.25},
    units::meters_per_second_t{ySpeed * 0.25},
    units::radians_per_second_t{rotation * 0.25}
};

    auto states = drive.ToSwerveModuleStates(speeds);
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&states, units::meters_per_second_t(4.5));
    frontLeft.SetDesiredState(states[0]);
    frontRight.SetDesiredState(states[1]);
    backLeft.SetDesiredState(states[2]);
    backRight.SetDesiredState(states[3]);

    frc::SmartDashboard::PutNumber("front left speed", states[0].speed.value());
    frc::SmartDashboard::PutNumber("front right speed", states[1].speed.value());
    frc::SmartDashboard::PutNumber("back left speed", states[2].speed.value());
    frc::SmartDashboard::PutNumber("back right speed", states[3].speed.value());

    frc::SmartDashboard::PutNumber("front left ang", states[0].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("front right ang", states[1].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("back left ang", states[2].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("back right ang", states[3].angle.Degrees().value());

    frc::SmartDashboard::PutNumber(
        "FL Actual Angle",
        frontLeft.GetCurrentAngle());

    frc::SmartDashboard::PutNumber(
        "FR Actual Angle",
        frontRight.GetCurrentAngle());

    frc::SmartDashboard::PutNumber(
        "BL Actual Angle",
        backLeft.GetCurrentAngle());

    frc::SmartDashboard::PutNumber(
        "BR Actual Angle",
        backRight.GetCurrentAngle());

    frc::SmartDashboard::PutNumber(
        "FL Actual SPEED",
        frontLeft.GetCurrentSpeed());

    frc::SmartDashboard::PutNumber(
        "FR Actual SPEED",
        frontRight.GetCurrentSpeed());

    frc::SmartDashboard::PutNumber(
        "BL Actual SPEED",
        backLeft.GetCurrentSpeed());

    frc::SmartDashboard::PutNumber(
        "BR Actual SPEED",
        backRight.GetCurrentSpeed());       
}

