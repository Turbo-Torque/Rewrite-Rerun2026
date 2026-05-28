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
    SwerveModule m_frontLeft{DriveConstants::kFrontLeftDriveID, DriveConstants::kFrontLeftTurnID};
    SwerveModule m_frontRight{DriveConstants::kFrontRightDriveID, DriveConstants::kFrontRightTurnID};
    SwerveModule m_backLeft{DriveConstants::kBackLeftDriveID, DriveConstants::kBackLeftTurnID};
    SwerveModule m_backRight{DriveConstants::kBackRightDriveID, DriveConstants::kBackRightTurnID};

    frc::SlewRateLimiter<units::scalar> xLimiter{3.0};
    frc::SlewRateLimiter<units::scalar> yLimiter{3.0};
    frc::SlewRateLimiter<units::scalar> rotationLimiter{3.0};

    frc::SwerveDriveKinematics drive{
        frc::Translation2d(0.381, 0.381),
        frc::Translation2d(0.381, -0.381),
        frc::Translation2d(-0.381, 0.381),
        frc::Translation2d(-0.381, -0.381)
    };
};