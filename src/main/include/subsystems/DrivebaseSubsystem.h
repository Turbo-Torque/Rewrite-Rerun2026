#pragma once

#include "frc2/command/SubsystemBase.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "ctre/phoenix6/CANBus.hpp"
#include "frc/controller/PIDController.h"
#include "networktables/BooleanTopic.h"
#include "networktables/StructTopic.h"
#include "networktables/StructArrayTopic.h"
#include <frc2/command/CommandPtr.h>
#include <functional>
#include <array>
#include "ctre/phoenix6/Pigeon2.hpp"
#include "turbolib/motors/NeoKrakenModule.hpp"
#include "frc/filter/SlewRateLimiter.h"


class DrivebaseSubsystem final:public frc2::SubsystemBase {
    public: 
    DrivebaseSubsystem();

    void Drive(const frc::ChassisSpeeds& speeds);
    void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states);
    frc::Rotation2d GetGyroAngle();
    void ZeroGyro();
    frc::Pose2d GetPose();

    std::array<frc::SwerveModuleState, 4> GetModuleStates();
    std::array<frc::SwerveModulePosition, 4> GetSwerveModulePosition();

    void Periodic() override;
    void SimulationPeriodic() override;

    frc2::CommandPtr DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rotationSpeed);

    private:
    ctre::phoenix6::CANBus canBus{};

    turbolib::motors::NeoKrakenModule frontLeft, frontRight, backLeft, backRight;
    ctre::phoenix6::hardware::Pigeon2 gyro;
    
    frc::PIDController rotationController{0.009, 0, 0};

    frc::ChassisSpeeds cmdSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s};

    nt::StructPublisher<frc::Pose2d> posePublisher;

    frc::SlewRateLimiter<units::meters_per_second> xSpeedLimiter{DriveConstants::kMaxLinearSpeed};
    frc::SlewRateLimiter<units::meters_per_second> ySpeedLimiter{DriveConstants::kMaxLinearSpeed};
    frc::SlewRateLimiter<units::radians_per_second> rotSpeedLimiter{DriveConstants::kMaxAngularSpeed};



};