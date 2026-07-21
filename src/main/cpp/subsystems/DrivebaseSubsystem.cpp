#include "subsystems/DrivebaseSubsystem.h"

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

using namespace DriveConstants;


DrivebaseSubsystem::DrivebaseSubsystem():
    frontLeft("FrontLeft", kFrontLeftPorts.driveMotorPort, kFrontLeftPorts.steerMotorPort, kFrontLeftPorts.encoderPort),
    frontRight("FrontRight", kFrontRightPorts.driveMotorPort, kFrontRightPorts.steerMotorPort, kFrontRightPorts.encoderPort),
    backLeft("BackLeft", kBackLeftPorts.driveMotorPort, kBackLeftPorts.steerMotorPort, kBackLeftPorts.encoderPort),
    backRight("BackRight", kBackRightPorts.driveMotorPort, kBackRightPorts.steerMotorPort, kBackRightPorts.encoderPort),
    poseEstimator(kKinematics, frc::Rotation2d{}, std::array<frc::SwerveModulePosition, 4>{}, frc::Pose2d{}){
        rotationController.SetTolerance(0.5);
        rotationController.EnableContinuousInput(0, 360);

        ConfigureTelemetry();
        SetName("DrivebaseSubsystem");
    }



void DrivebaseSubsystem::Drive(const frc::ChassisSpeeds& speeds){
    auto states = DriveConstants::kKinematics.ToSwerveModuleStates(speeds);
    cmdSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, GetGyroAngle());
    SetModuleStates(states);
}

void DrivebaseSubsystem::ConfigureTelemetry() {
    posePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("DrivebaseSubsystem/Pose").Publish();
    cmdSpeedsPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("DriveSubsystem/CmdSpeeds").Publish();
    swerveModuleStatePublisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("DriveSubsystem/SwerveStates").Publish();
    gyroPublisher =
    nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Rotation2d>("DriveSubsystem/Gyro").Publish();

    for (auto& [name, pose] : DriveConstants::kStartingPose) {
        startingPoseChooser.AddOption(name, name);
    }
    startingPoseChooser.SetDefaultOption("Center", "Center");
    frc::SmartDashboard::PutData("Starting Pose", &startingPoseChooser);
    frc::SmartDashboard::PutData("Field", &field);

}

void DrivebaseSubsystem::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
    frontLeft.SetModuleState(states[0]);
    frontRight.SetModuleState(states[1]);
    backLeft.SetModuleState(states[2]);
    backRight.SetModuleState(states[3]);
}

void DrivebaseSubsystem::ZeroGyro() {
    gyro.Reset();

    poseEstimator.ResetPosition(
        GetGyroAngle(),
        GetSwerveModulePosition(),
        frc::Pose2d{});
}



void DrivebaseSubsystem::ResetPose(frc::Pose2d pose) {
    poseEstimator.ResetPosition(GetGyroAngle(), GetSwerveModulePosition(), pose);
    simPose = pose;
}

void DrivebaseSubsystem::ApplyStartingPose() {
    auto pose = DriveConstants::kStartingPose.find(startingPoseChooser.GetSelected());
    if (pose != DriveConstants::kStartingPose.end()) {
        ResetPose(pose->second);
    }
}

frc2::CommandPtr DrivebaseSubsystem::DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rotSpeed) {
    return frc2::FunctionalCommand ( []{},
        [=, this] {

            //double x = frc::ApplyDeadband(xSpeed(), DriveConstants::kControllerDeadBand);
            // double y = frc::ApplyDeadband(ySpeed(), DriveConstants::kControllerDeadBand);
            //double rot = frc::ApplyDeadband(rotSpeed(), DriveConstants::kControllerDeadBand);

            const frc::ChassisSpeeds speeds{

             
                    - (xSpeed() * DriveConstants::kMaxLinearSpeed),
                    - (ySpeed() * DriveConstants::kMaxLinearSpeed),
                    - (rotSpeed() * DriveConstants::kMaxAngularSpeed)
            };
            frc::ChassisSpeeds cmdspeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetGyroAngle());
            Drive(cmdspeeds);
        },
        [this](bool) { Drive(frc::ChassisSpeeds{}); }, [] {return false;}, {this}
    ).ToPtr().WithName("Drive");
}

frc::Rotation2d DrivebaseSubsystem::GetGyroAngle() {
    return gyro.GetRotation2d();
}

frc::Pose2d DrivebaseSubsystem::GetPose() {
    //return poseEstimator.GetEstimatedPosition();
    return simPose;
}

frc::Rotation2d DrivebaseSubsystem::GetAngle() {
    return gyro.GetRotation2d();
}

std::array<frc::SwerveModuleState, 4> DrivebaseSubsystem::GetModuleStates(){
    return {frontLeft.GetModuleState(), frontRight.GetModuleState(), backLeft.GetModuleState(), backRight.GetModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DrivebaseSubsystem::GetSwerveModulePosition() {
    return {frontLeft.GetModulePosition(), frontRight.GetModulePosition(), backLeft.GetModulePosition(), backRight.GetModulePosition()};
}


void DrivebaseSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Gyro", GetGyroAngle().Degrees().value());
    poseEstimator.Update(GetGyroAngle(), GetSwerveModulePosition());
    posePublisher.Set(GetPose());
    cmdSpeedsPublisher.Set(cmdSpeeds);
    swerveModuleStatePublisher.Set(GetModuleStates());
    gyroPublisher.Set(GetGyroAngle());
    field.SetRobotPose(GetPose());
}

void DrivebaseSubsystem::SimulationPeriodic() {
    constexpr units::second_t kSimPeriod = 20_ms;

    auto [vx, vy, omega] = cmdSpeeds;
    const auto newX = simPose.X() + vx * kSimPeriod;
    const auto newY = simPose.Y() + vy * kSimPeriod;
    const auto newMega = frc::Rotation2d(simPose.Rotation().Radians() + omega * kSimPeriod);
    simPose = frc::Pose2d(newX, newY, newMega);
    field.SetRobotPose(GetPose());

}