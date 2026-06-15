#pragma once

#include "frc/geometry/Rotation2d.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

class SwerveModule {
public:
    SwerveModule (
        int driveMotorID,
        int turnMotorID,
        int encoderID);

    frc::SwerveModuleState GetState();

    void SetDesiredState(const frc::SwerveModuleState& state);

    void SetDriveInverted(bool inverted);

    void PublishInfo(const std::string& name, double num) {
        frc::SmartDashboard::PutNumber(name, num);
    }

    double GetCurrentAngle();

private:
    ctre::phoenix6::hardware::TalonFX driveMotor;
    rev::spark::SparkMax turnMotor;
    ctre::phoenix6::hardware::CANcoder turnEncoder;
    frc::PIDController turnPIDController{0.015, 0.0, 0.0001};
    
};
