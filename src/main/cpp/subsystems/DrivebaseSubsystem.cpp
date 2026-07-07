#include "subsystems/DrivebaseSubsystem.h"

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>

using namespace DriveConstants;


DrivebaseSubsystem::DrivebaseSubsystem():
    frontLeft("FrontLeft", kFrontLeftPorts.driveMotorPort, kFrontLeftPorts.steerMotorPort, kFrontLeftPorts.encoderPort),
    frontRight("FrontRight", kFrontRightPorts.driveMotorPort, kFrontRightPorts.steerMotorPort, kFrontRightPorts.encoderPort),
    backLeft("BackLeft", kBackLeftPorts.driveMotorPort, kBackLeftPorts.steerMotorPort, kBackLeftPorts.encoderPort),
    backRight("BackRight", kBackRightPorts.driveMotorPort, kBackRightPorts.steerMotorPort, kBackRightPorts.encoderPort),
    gyro(DriveConstants::kGyro){
        rotationController.SetTolerance(0.5);
        rotationController.EnableContinuousInput(0, 360);
    }

void DrivebaseSubsystem::Drive(const frc::ChassisSpeeds& speeds){
    auto states = DriveConstants::kKinematics.ToSwerveModuleStates(speeds);
    cmdSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, GetGyroAngle());
    SetModuleStates(states);
}

void DrivebaseSubsystem::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
    frontLeft.SetModuleState(states[0]);
    frontRight.SetModuleState(states[1]);
    backLeft.SetModuleState(states[2]);
    backRight.SetModuleState(states[3]);
}

frc2::CommandPtr DrivebaseSubsystem::DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rotSpeed) {
    return frc2::FunctionalCommand ( []{},
        [=, this] {

            double x = frc::ApplyDeadband(xSpeed(), DriveConstants::kControllerDeadBand);
            double y = frc::ApplyDeadband(ySpeed(), DriveConstants::kControllerDeadBand);
            double rot = frc::ApplyDeadband(rotSpeed(), DriveConstants::kControllerDeadBand);

            const frc::ChassisSpeeds speeds{

             
                    -x * DriveConstants::kMaxLinearSpeed,
                    y * DriveConstants::kMaxLinearSpeed,
                    rot * DriveConstants::kMaxAngularSpeed
            };
            //cmd speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetGyroAngle());
            Drive(speeds);
        },
        [this](bool) { Drive(frc::ChassisSpeeds{}); }, [] {return false;}, {this}
    ).ToPtr().WithName("Drive");
}

frc::Rotation2d DrivebaseSubsystem::GetGyroAngle() {
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
}

void DrivebaseSubsystem::ZeroGyro() {
    gyro.Reset();
}

void DrivebaseSubsystem::SimulationPeriodic() {
    
}