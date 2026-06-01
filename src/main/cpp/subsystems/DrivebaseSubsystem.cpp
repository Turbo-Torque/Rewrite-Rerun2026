#include "subsystems/DrivebaseSubsystem.h"

#include <frc/kinematics/ChassisSpeeds.h>

DrivebaseSubsystem::DrivebaseSubsystem() {
};

void DrivebaseSubsystem::Drive(double xSpeed, double ySpeed, double rotation) {
    frc::ChassisSpeeds speeds{    
    units::meters_per_second_t{xSpeed},
    units::meters_per_second_t{ySpeed},
    units::radians_per_second_t{rotation}
};

    auto states = drive.ToSwerveModuleStates(speeds);
    frontLeft.SetDesiredState(states[0]);
    frontRight.SetDesiredState(states[1]);
    backLeft.SetDesiredState(states[2]);
    backRight.SetDesiredState(states[3]);
}