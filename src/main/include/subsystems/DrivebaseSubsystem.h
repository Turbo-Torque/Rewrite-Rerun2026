#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <frc/filter/SlewRateLimiter.h>
#include "subsystems/SwerveModule.h"
#include "Constants.h"

class DrivebaseSubsystem : public frc2::SubsystemBase {
 public:
  DrivebaseSubsystem();

  void Drive(double xSpeed, double ySpeed, double rotation);


 private:
    SwerveModule frontLeft{DriveConstants::kFrontLeftDriveID, DriveConstants::kFrontLeftTurnID, DriveConstants::kFrontLeftEncoderID};
    SwerveModule frontRight{DriveConstants::kFrontRightDriveID, DriveConstants::kFrontRightTurnID, DriveConstants::kFrontRightEncoderID};
    SwerveModule backLeft{DriveConstants::kBackLeftDriveID, DriveConstants::kBackLeftTurnID, DriveConstants::kBackLeftEncoderID};
    SwerveModule backRight{DriveConstants::kBackRightDriveID, DriveConstants::kBackRightTurnID, DriveConstants::kBackRightEncoderID};

    frc::SlewRateLimiter<units::scalar> xLimiter{3.0 / 1_s};
    frc::SlewRateLimiter<units::scalar> yLimiter{3.0 / 1_s};
    frc::SlewRateLimiter<units::scalar> rotationLimiter{3.0 / 1_s};

    frc::SwerveDriveKinematics<4> drive{
        frc::Translation2d(0.381_m, 0.381_m),
        frc::Translation2d(0.381_m, -0.381_m),
        frc::Translation2d(-0.381_m, 0.381_m),
        frc::Translation2d(-0.381_m, -0.381_m)
    };
};