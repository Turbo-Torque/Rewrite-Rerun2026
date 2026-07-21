

#pragma once

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include <turbolib/structure/SwervePorts.hpp>
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"
#include "frc/geometry/Pose2d.h"
#include <map>
#include <string>


using namespace units::literals;

namespace OperatorConstants {

    inline constexpr int kDriveControllerPort = 0;
    inline constexpr int kOperatorControllerPort = 1;

}

namespace DriveConstants {
    inline turbolib::structure::SwervePorts kBackRightPorts{5, 6, 20};
    inline turbolib::structure::SwervePorts kFrontRightPorts{7, 8, 21};
    inline turbolib::structure::SwervePorts kFrontLeftPorts{1, 2, 18};
    inline turbolib::structure::SwervePorts kBackLeftPorts{3, 4, 19};
    inline constexpr int kGyro = 22;

    inline constexpr double kRobotLength = 0.5525;
    inline constexpr double kRobotWidth = 0.5525;

    inline constexpr frc::Translation2d kModulePosition[] = {
        {units::meter_t{kRobotLength / 2}, units::meter_t{kRobotWidth / 2}},   
        {units::meter_t{kRobotLength / 2}, units::meter_t{-kRobotWidth / 2}},  
        {units::meter_t{-kRobotLength / 2}, units::meter_t{kRobotWidth / 2}},  
        {units::meter_t{-kRobotLength / 2}, units::meter_t{-kRobotWidth / 2}}  
    }; 

    inline frc::SwerveDriveKinematics<4> kKinematics{kModulePosition[0], kModulePosition[1], kModulePosition[2], kModulePosition[3]};


    inline constexpr units::meters_per_second_t kMaxLinearSpeed = 4.25_mps;
    inline constexpr units::radians_per_second_t kMaxAngularSpeed = 570_deg_per_s;
    inline constexpr double kControllerDeadBand = 0.08;

    inline const std::map<std::string, frc::Pose2d> kStartingPose = {
        {"bLeft", frc::Pose2d(7.5_m, 6.5_m, 180_deg)},
        {"bCenter", frc::Pose2d(7.5_m, 4.0_m, 180_deg)},
        {"bRight", frc::Pose2d(7.5_m, 1.5_m, 180_deg)}
    };
}

namespace IntakeConstants {
    inline constexpr int kIntakeMotorPort = 10;
    inline constexpr int kIntakePivotPort = 9;
    //inline constexpr int kIntakeCANPort = 0;

    inline constexpr double kIntakeUp = 0.0;
    inline constexpr double kIntakeDown = 57.0;
    inline constexpr double kFFPivot = 0.0;
    inline constexpr units::volt_t kIntakeVolts = 5_V;

}

namespace HopperConstants {
    inline constexpr int kHopperMotorPort = 11;
    inline constexpr units::volt_t kHopperInVolts = 5_V;
    //inline constexpr units::volt_t kHopperOutVolts = -3_V;
}

namespace GateConstants {
    inline constexpr int kRightGateMotorPort = 15;
    inline constexpr int kLeftGateMotorPort = 14;
    inline constexpr units::volt_t kGateVolts = 5_V;
}


namespace ShooterConstants {
    inline constexpr int kRightShooterMotorPort = 6;
    inline constexpr int kLeftShooterMotorPort = 16;
    inline constexpr units::revolutions_per_minute_t kShooterRPM = 2000_rpm;
}

namespace RobotConstants{
    inline constexpr double kStartVoltage = 11;
}
