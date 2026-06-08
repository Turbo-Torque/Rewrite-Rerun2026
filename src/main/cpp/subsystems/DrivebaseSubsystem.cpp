#include "subsystems/DrivebaseSubsystem.h"

#include <frc/kinematics/ChassisSpeeds.h>

DrivebaseSubsystem::DrivebaseSubsystem() {};

void DrivebaseSubsystem::Drive(double xSpeed, double ySpeed, double rotation) {
    frc::ChassisSpeeds speeds{    
    units::meters_per_second_t{xSpeed * 0.25},
    units::meters_per_second_t{ySpeed * 0.25},
    units::radians_per_second_t{rotation * 0.25}
};

    auto states = drive.ToSwerveModuleStates(speeds);
    frontLeft.SetDesiredState(states[0]);
    frontRight.SetDesiredState(states[1]);
    backLeft.SetDesiredState(states[2]);
    backRight.SetDesiredState(states[3]);
}