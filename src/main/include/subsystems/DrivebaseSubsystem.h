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
#include "turbolib/perception/TurboPoseEstimator.hpp"
#include "frc/filter/SlewRateLimiter.h"
#include "Constants.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>



class DrivebaseSubsystem final:public frc2::SubsystemBase {
    public: 
    DrivebaseSubsystem();



    void Drive(const frc::ChassisSpeeds& speeds);
    void AutoDrive(const frc::ChassisSpeeds& speeds);
    void ConfigureTelemetry();
    void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states);
    void ZeroGyro();
    void ResetPose(frc::Pose2d pose);
    void ApplyStartingPose();

    turbolib::perception::TurboPoseEstimator& GetPoseEstimator();

    frc::Pose2d GetPose();
    frc::Rotation2d GetGyroAngle();
    frc::Rotation2d GetAngle();
    std::array<frc::SwerveModuleState, 4> GetModuleStates();
    std::array<frc::SwerveModulePosition, 4> GetSwerveModulePosition();

    frc2::CommandPtr DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rotationSpeed);
    

    void Periodic() override;
    void SimulationPeriodic() override;

    private:
    ctre::phoenix6::CANBus canBus{};

    turbolib::motors::NeoKrakenModule frontLeft, frontRight, backLeft, backRight;
    ctre::phoenix6::hardware::Pigeon2 gyro{DriveConstants::kGyro, canBus};
    frc::SwerveDrivePoseEstimator<4> poseEstimator;
    
    frc::PIDController rotationController{0.009, 0, 0};

    frc::ChassisSpeeds cmdSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s};
    frc::Pose2d simPose = frc::Pose2d(9_m, 4_m, 0_deg);

    frc::SendableChooser<std::string> startingPoseChooser;
    frc::Field2d field;

    nt::StructPublisher<frc::Pose2d> posePublisher;
    nt::StructPublisher<frc::ChassisSpeeds> cmdSpeedsPublisher;
    nt::StructArrayPublisher<frc::SwerveModuleState> swerveModuleStatePublisher;
    nt::StructPublisher<frc::Rotation2d> gyroPublisher;

    // frc::SlewRateLimiter<units::meters_per_second> xSpeedLimiter{DriveConstants::kMaxLinearSpeed};
    // frc::SlewRateLimiter<units::meters_per_second> ySpeedLimiter{DriveConstants::kMaxLinearSpeed};
    // frc::SlewRateLimiter<units::radians_per_second> rotSpeedLimiter{DriveConstants::kMaxAngularSpeed};



};