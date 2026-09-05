#include "subsystems/DrivebaseSubsystem.h"

#include "Constants.h"
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "RobotContainer.h"
#include "frc/RobotController.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/FunctionalCommand.h"
#include "turbolib/perception/TurboPoseEstimator.hpp"
#include "units/length.h"
#include "units/velocity.h"
#include <functional>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <algorithm>
#include "RobotContainer.h"


using namespace DriveConstants;


DrivebaseSubsystem::DrivebaseSubsystem():
    frontLeft("FrontLeft", kFrontLeftPorts.driveMotorPort, kFrontLeftPorts.steerMotorPort, kFrontLeftPorts.encoderPort),
    frontRight("FrontRight", kFrontRightPorts.driveMotorPort, kFrontRightPorts.steerMotorPort, kFrontRightPorts.encoderPort),
    backLeft("BackLeft", kBackLeftPorts.driveMotorPort, kBackLeftPorts.steerMotorPort, kBackLeftPorts.encoderPort),
    backRight("BackRight", kBackRightPorts.driveMotorPort, kBackRightPorts.steerMotorPort, kBackRightPorts.encoderPort),
    poseEstimator(frc::Rotation2d{}, std::array<frc::SwerveModulePosition, 4>{}, frc::Pose2d{}, kKinematics){
        realRotationController.SetTolerance(0.5_deg, 30_deg_per_s);
        realRotationController.EnableContinuousInput(0_deg, 360_deg);
        simRotationController.SetTolerance(0.5_deg, 30_deg_per_s);
        simRotationController.EnableContinuousInput(0_deg, 360_deg);
        ConfigureAutoBuilder();
        ConfigureEstimator();
        ConfigureTelemetry();

        
        SetName("DrivebaseSubsystem");
    }

void DrivebaseSubsystem::ConfigureAutoBuilder() {
    auto config = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this] { return GetPose();},
        [this](frc::Pose2d pose) {ResetPose(pose);},
        [this] {return GetRobotRelativeSpeeds();},
        [this](auto speeds) { AutoDrive(speeds);},
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants(AutoConstants::kTranslationP, AutoConstants::kTranslationI, AutoConstants::kTranslationD),
            pathplanner::PIDConstants(AutoConstants::kRotationP, AutoConstants::kRotationI, AutoConstants::kRotationD)
        ),
        config,
        [] {
            auto alliance = frc::DriverStation::GetAlliance();
            return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
            
        },
    this);
}

void DrivebaseSubsystem::ConfigureEstimator() {
    poseEstimator.ResetEstimatorPosition(GetGyroAngle(), GetSwerveModulePosition(), frc::Pose2d{});
    // poseEstimator.AddLocalizationCamera("LSCam", {3.5_in, 10.5_in, 29.6_in, frc::Rotation3d{0_rad, -30_deg, 0_rad}},
    //                               frc::AprilTagField::k2026RebuiltAndyMark);
    //  poseEstimator.AddLocalizationCamera("rightShooterCam",
    //                               frc::Transform3d{3.7_in, -10.5_in, 29.6_in, frc::Rotation3d{0_deg, -30_deg, 0_deg}},
    //                               frc::AprilTagField::k2026RebuiltAndyMark);
    // poseEstimator.AddLocalizationCamera("blCam", frc::Transform3d{-10.477_in, 10.379_in, 6.576_in, frc::Rotation3d{0_deg, -22.23_deg, -260_deg}}, frc::AprilTagField::k2026RebuiltAndyMark);
        
    
}

void DrivebaseSubsystem::ConfigureTelemetry() {
    posePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("DrivebaseSubsystem/Pose").Publish();
    cmdSpeedsPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("DriveSubsystem/CmdSpeeds").Publish();
    swerveModuleStatePublisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("DriveSubsystem/SwerveStates").Publish();
    gyroPublisher =
    nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Rotation2d>("DriveSubsystem/Gyro").Publish();
    seesTagPublisher = nt::NetworkTableInstance::GetDefault().GetBooleanTopic("DriveSubsystem/SeesTag").Publish();

    frc::SmartDashboard::PutData("Field", &field);
}



void DrivebaseSubsystem::Drive(const frc::ChassisSpeeds& speeds){
    auto states = DriveConstants::kKinematics.ToSwerveModuleStates(speeds);
    cmdSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, GetGyroAngle());
    SetModuleStates(states);
}

void DrivebaseSubsystem::AutoDrive(const frc::ChassisSpeeds& speeds) {
    auto x = speeds.vx * 1;
    auto y = speeds.vy * 1;
    auto rot = speeds.omega * 1;

    auto states = DriveConstants::kKinematics.ToSwerveModuleStates({x, y, rot});

    if (frc::RobotBase::IsReal()) {
        SetModuleStates(states);
    }
}


void DrivebaseSubsystem::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
    frontLeft.SetModuleState(states[0]);
    frontRight.SetModuleState(states[1]);
    backLeft.SetModuleState(states[2]);
    backRight.SetModuleState(states[3]);
}

void DrivebaseSubsystem::ZeroGyro() {
    gyro.Reset();

    poseEstimator.ResetEstimatorPosition(
        GetGyroAngle(),
        GetSwerveModulePosition(),
        frc::Pose2d{});
}

turbolib::perception::TurboPoseEstimator& DrivebaseSubsystem::GetPoseEstimator() {
    return poseEstimator;
}

void DrivebaseSubsystem::ResetPose(frc::Pose2d pose) {
    poseEstimator.ResetEstimatorPosition(GetGyroAngle(), GetSwerveModulePosition(), pose);
    simPose = pose;
}
void DrivebaseSubsystem::SelectCenterSetpoint() {
    selectSetpoint = IsRedAlliance() ? PathingConstants::kRedCenter : PathingConstants::kBlueCenter;
}

void DrivebaseSubsystem::SelectLeftSetpoint() {
    selectSetpoint = IsRedAlliance() ? PathingConstants::kRedTop : PathingConstants::kBlueBottom;
}

void DrivebaseSubsystem::SelectRightSetpoint() {
    selectSetpoint = IsRedAlliance() ? PathingConstants::kRedBottom : PathingConstants::kRedBottom;
}

frc::ProfiledPIDController<units::degrees>& DrivebaseSubsystem::ActiveRotationController() {
    return frc::RobotBase::IsSimulation() ? simRotationController : realRotationController;
}

bool DrivebaseSubsystem::AtHeadingSetpoint() {
    return ActiveRotationController().AtSetpoint();
}

bool DrivebaseSubsystem::IsRedAlliance() {
    auto alliance = frc::DriverStation::GetAlliance();
    return alliance == frc::DriverStation::Alliance::kRed;
}

bool DrivebaseSubsystem::SeesTag() {
    if (poseEstimator.SeesTag()) {
        return true;
    }
    return false;
}

// ADDED: was declared but never defined, so DriveToSetpointCommand couldn't link
bool DrivebaseSubsystem::AtPoseSetPoint() {
    if ((GetPose().Translation().Distance(selectSetpoint.Translation()) < DriveConstants::kSetpointTolerance) && AtHeadingSetpoint()){
        return true;
    }
    return false;
}

frc2::CommandPtr DrivebaseSubsystem::DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rotSpeed) {
    return frc2::FunctionalCommand ( []{},
        [=, this] {

            double xInput = frc::ApplyDeadband(xSpeed(), DriveConstants::kControllerDeadBand);
            double yInput = frc::ApplyDeadband(ySpeed(), DriveConstants::kControllerDeadBand);
            double rotInput = frc::ApplyDeadband(rotSpeed(), DriveConstants::kControllerDeadBand);

            units::meters_per_second_t x;
            units::meters_per_second_t y;
            units::radians_per_second_t rot;

            if (IsRedAlliance()) {
                
                x =  xInput * DriveConstants::kMaxLinearSpeed * 0.75;
                y =   yInput * DriveConstants::kMaxLinearSpeed * 0.75;
                rot = rotInput * DriveConstants::kMaxAngularSpeed * 0.65;
            } else {
                x = xInput * DriveConstants::kMaxLinearSpeed * 0.75;
                y =  yInput * DriveConstants::kMaxLinearSpeed * 0.75;
                rot = rotInput * DriveConstants::kMaxAngularSpeed * 0.65;
            }

            const frc::ChassisSpeeds speeds{x, y, rot};
            frc::ChassisSpeeds cmdspeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetGyroAngle());
            Drive(cmdspeeds);
        },
        [this](bool) { Drive(frc::ChassisSpeeds{}); }, [] {return false;}, {this}
    ).ToPtr().WithName("Drive");
}

frc2::CommandPtr DrivebaseSubsystem::RotateToHubCommand(std::function<frc::Rotation2d()> angle) {
    return frc2::FunctionalCommand([] {},
        [=, this] {
            const auto currentAngle = GetPose().Rotation().Degrees();
            const auto targetAngle = angle().Degrees();

            double rotSpeed = ActiveRotationController().Calculate(currentAngle, targetAngle);
            units::degrees_per_second_t maxSpeed{DriveConstants::kMaxAngularSpeed};
            rotSpeed = std::clamp(rotSpeed, -maxSpeed.value(), maxSpeed.value());

            Drive(frc::ChassisSpeeds{0_mps, 0_mps, units::degrees_per_second_t{rotSpeed}});
        },
        [this](bool) { Drive(frc::ChassisSpeeds{}); },
        [=, this] { return AtHeadingSetpoint(); },
        {this}
    ).ToPtr().WithName("Rotate To Hub");
}


frc2::CommandPtr DrivebaseSubsystem::GetAngletoHubCommand(){
    std::function<frc::Rotation2d()> bestTargetAngle = [this]() {
        frc::Pose2d targetPose = (IsRedAlliance()) ? PathingConstants::kRedHubPose : PathingConstants::kBlueHubPose;

        frc::Pose2d currentPose = GetPose();

        frc::Translation2d targetPosition = (targetPose.Translation() - currentPose.Translation());
        auto targetAngle = std::atan2(targetPosition.Y().value(), targetPosition.X().value());

        return frc::Rotation2d(units::radian_t(targetAngle));
    };
    frc::SmartDashboard::PutNumber("best target angle", bestTargetAngle().Degrees().value());
    return RotateToHubCommand(bestTargetAngle);
}

// FIXED: signature (missing "()"), used xController/yController instead of the rotation controller,
// clamped output to kMaxLinearSpeed, closed the FunctionalCommand(...) call and added .ToPtr()
frc2::CommandPtr DrivebaseSubsystem::DriveToSetpointCommand(std::function<frc::Translation2d()> distance) {
    return frc2::FunctionalCommand([]{},
        [=, this] {
            const auto errorX = distance().X();
            const auto errorY = distance().Y();

            double driveXSpeed = xController.Calculate(0.0, errorX.value());
            double driveYSpeed = yController.Calculate(0.0, errorY.value());

            units::meters_per_second_t maxSpeed{DriveConstants::kMaxLinearSpeed};
            driveXSpeed = std::clamp(driveXSpeed, -maxSpeed.value(), maxSpeed.value());
            driveYSpeed = std::clamp(driveYSpeed, -maxSpeed.value(), maxSpeed.value());

            Drive(frc::ChassisSpeeds{units::meters_per_second_t{-driveXSpeed}, units::meters_per_second_t{-driveYSpeed}, 0_rad_per_s});
        },
        [this](bool) { Drive(frc::ChassisSpeeds{}); },
        [this] { return AtPoseSetPoint(); },
        {this}
    ).ToPtr().WithName("Drive To Setpoint");
}

frc2::CommandPtr DrivebaseSubsystem::GetPoseToSetpoint() {
    std::function<frc::Translation2d()> bestTargetDistance = [this]() {
        frc::Pose2d targetPose = selectSetpoint;
        frc::Pose2d currentPose = GetPose();

        units::meter_t targetX = (targetPose.X() - currentPose.X());
        units::meter_t targetY = (targetPose.Y() - currentPose.Y());

        units::meter_t targetZ{std::hypot(targetX.value(), targetY.value())};

        return frc::Translation2d(units::meter_t(targetX), units::meter_t(targetY));
    };

    return DriveToSetpointCommand(bestTargetDistance);
}



frc::Rotation2d DrivebaseSubsystem::GetGyroAngle() {
    return gyro.GetRotation2d().RotateBy(180_deg);
}

frc::Pose2d DrivebaseSubsystem::GetPose() {
    return poseEstimator.GetPose2D();
}

frc::Rotation2d DrivebaseSubsystem::GetAngle() {
    return gyro.GetRotation2d();
}

frc::ChassisSpeeds DrivebaseSubsystem::GetRobotRelativeSpeeds() {
    return DriveConstants::kKinematics.ToChassisSpeeds(GetModuleStates());
}

std::array<frc::SwerveModuleState, 4> DrivebaseSubsystem::GetModuleStates(){
    return {frontLeft.GetModuleState(), frontRight.GetModuleState(), backLeft.GetModuleState(), backRight.GetModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DrivebaseSubsystem::GetSwerveModulePosition() {
    return {frontLeft.GetModulePosition(), frontRight.GetModulePosition(), backLeft.GetModulePosition(), backRight.GetModulePosition()};
}


void DrivebaseSubsystem::Periodic() {
    posePublisher.Set(GetPose());
    cmdSpeedsPublisher.Set(cmdSpeeds);
    swerveModuleStatePublisher.Set(GetModuleStates());
    gyroPublisher.Set(GetGyroAngle());
    field.SetRobotPose(GetPose());
    seesTagPublisher.Set(poseEstimator.SeesTag());
    if constexpr (frc::RobotBase::IsReal()) {
    poseEstimator.UpdateWithOdometryAndVision(GetGyroAngle(), GetSwerveModulePosition());
    } else {
    poseEstimator.UpdateWithAllAvailableVisionMeasurements(GetGyroAngle());
    }

    frc::SmartDashboard::PutNumber("Gyro", GetGyroAngle().Degrees().value());
    auto alliance = frc::DriverStation::GetAlliance();
    frc::SmartDashboard::PutBoolean("HasAlliance", alliance.has_value());
    frc::SmartDashboard::PutBoolean("At Setpoint", AtPoseSetPoint());
    frc::SmartDashboard::PutBoolean("IsRed", alliance && alliance.value() == frc::DriverStation::Alliance::kRed);   
}

void DrivebaseSubsystem::SimulationPeriodic() {
    constexpr units::second_t kSimPeriod = 20_ms;

    auto [vx, vy, omega] = cmdSpeeds;
    const auto newX = simPose.X() + vx * kSimPeriod;
    const auto newY = simPose.Y() + vy * kSimPeriod;
    const auto newMega = frc::Rotation2d(simPose.Rotation().Radians() + omega * kSimPeriod);
    if (IsRedAlliance()) {
        simPose = frc::Pose2d(newX, newY, newMega);
    } else {
        simPose = frc::Pose2d(newX, newY, newMega);
    }


    

    gyro.GetSimState().SetRawYaw(simPose.Rotation().Degrees());

    poseEstimator.UpdateAllSims(simPose);
    poseEstimator.ResetEstimatorPosition(GetGyroAngle(), GetSwerveModulePosition(), simPose);

    field.SetRobotPose(GetPose());

}