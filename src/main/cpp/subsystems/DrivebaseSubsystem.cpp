#include "subsystems/DrivebaseSubsystem.h"

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "turbolib/perception/TurboPoseEstimator.hpp"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <algorithm>


using namespace DriveConstants;


DrivebaseSubsystem::DrivebaseSubsystem():
    frontLeft("FrontLeft", kFrontLeftPorts.driveMotorPort, kFrontLeftPorts.steerMotorPort, kFrontLeftPorts.encoderPort),
    frontRight("FrontRight", kFrontRightPorts.driveMotorPort, kFrontRightPorts.steerMotorPort, kFrontRightPorts.encoderPort),
    backLeft("BackLeft", kBackLeftPorts.driveMotorPort, kBackLeftPorts.steerMotorPort, kBackLeftPorts.encoderPort),
    backRight("BackRight", kBackRightPorts.driveMotorPort, kBackRightPorts.steerMotorPort, kBackRightPorts.encoderPort),
    poseEstimator(frc::Rotation2d{}, std::array<frc::SwerveModulePosition, 4>{}, frc::Pose2d{}, kKinematics){
        realRotationController.SetTolerance(0.5);
        realRotationController.EnableContinuousInput(0, 360);
        simRotationController.SetTolerance(0.5);
        simRotationController.EnableContinuousInput(0, 360);
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
    poseEstimator.AddLocalizationCamera("LSCam", {3.5_in, 10.5_in, 29.6_in, frc::Rotation3d{0_rad, -30_deg, 0_rad}},
                                  frc::AprilTagField::k2026RebuiltAndyMark);
    poseEstimator.AddLocalizationCamera("rightShooterCam",
                                  frc::Transform3d{3.5_in, -10.5_in, 29.6_in, frc::Rotation3d{0_deg, -30_deg, 0_deg}},
                                  frc::AprilTagField::k2026RebuiltAndyMark);
    poseEstimator.AddLocalizationCamera("blCam", frc::Transform3d{-10.477_in, 10.379_in, 6.576_in, frc::Rotation3d{0_deg, -22.23_deg, -135_deg}}, frc::AprilTagField::k2026RebuiltAndyMark);
        
    
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
    auto x = speeds.vx * -1;
    auto y = speeds.vy * -1;
    auto rot = speeds.omega * -1;

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

frc::PIDController& DrivebaseSubsystem::ActiveRotationController() {
    return frc::RobotBase::IsSimulation() ? simRotationController : realRotationController;
}

void DrivebaseSubsystem::AimAtHeading(frc::Rotation2d targetHeading) {

    double output = ActiveRotationController().Calculate(GetPose().Rotation().Degrees().value(), targetHeading.Degrees().value());


    units::degrees_per_second_t maxSpeed{DriveConstants::kMaxAngularSpeed};
    output = std::clamp(output, -maxSpeed.value(), maxSpeed.value());

    Drive(frc::ChassisSpeeds{0_mps, 0_mps, units::degrees_per_second_t{output}});
}

bool DrivebaseSubsystem::AtHeadingSetpoint() {
    ActiveRotationController().SetTolerance(0.5, 30.0);
    return ActiveRotationController().AtSetpoint();
}

bool DrivebaseSubsystem::IsRedAlliance() {
    auto alliance = frc::DriverStation::GetAlliance();
    return alliance == frc::DriverStation::Alliance::kRed;
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
                
                x = DriveConstants::negativeMetersConvert * xInput * DriveConstants::kMaxLinearSpeed * 0.75;
                y =  DriveConstants::negativeMetersConvert * yInput * DriveConstants::kMaxLinearSpeed * 0.75;
                rot = DriveConstants::negativeTurnConvert * rotInput * DriveConstants::kMaxAngularSpeed * 0.50;
                rot = rotInput * DriveConstants::kMaxAngularSpeed;
            } else {
                x = xInput * DriveConstants::kMaxLinearSpeed * 0.75;
                y =  yInput * DriveConstants::kMaxLinearSpeed * 0.75;
                rot = rotInput * DriveConstants::kMaxAngularSpeed * 0.50;
            }

            const frc::ChassisSpeeds speeds{x, y, rot};
            frc::ChassisSpeeds cmdspeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetGyroAngle());
            Drive(cmdspeeds);
        },
        [this](bool) { Drive(frc::ChassisSpeeds{}); }, [] {return false;}, {this}
    ).ToPtr().WithName("Drive");
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