#include "subsystems/DrivebaseSubsystem.h"

#include <frc/kinematics/ChassisSpeeds.h>

DrivebaseSubsystem::DrivebaseSubsystem() {
};

void DrivebaseSubsystem::Drive(double xlimiter.Calculate(xSpeed), double yLimiter.Calculate(ySpeed), double rotationLimiter.Calculate(rotation)) {
    frc::ChassisSpeed speed{xSpeed, ySpeed, rotation};

    auto states = drive.toSwerveModuleStates(speeds);
    frontLeft.SetDesiredState(states[0]);
    frontRight.SetDesiredState(states[1]);
    backLeft.SetDesiredState(states[2]);
    backRight.SetDesiredState(states[3]);
}