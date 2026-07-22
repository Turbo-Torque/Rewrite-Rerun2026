

#pragma once

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include <turbolib/structure/SwervePorts.hpp>
#include "units/acceleration.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"
#include "frc/geometry/Pose2d.h"
#include <map>
#include <string>
#include "frc/geometry/Translation3d.h"


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
        {"bLeft", frc::Pose2d(3.5_m, 5.5_m, 270_deg)},
        {"bCenter", frc::Pose2d(3.5_m, 4.0_m, 180_deg)},
        {"bRight", frc::Pose2d(3.5_m, 1.5_m, 90_deg)},
        {"rLeft", frc::Pose2d(12.5_m, 6_m, 270_deg)},
        {"rCenter", frc::Pose2d(12.5_m, 4.0_m, 180_deg)},
        {"rRight", frc::Pose2d(12.5_m, 1.5_m, 90_deg)}
    };
}

namespace AutoConstants {
    inline constexpr double kTranslationP = 5.0;
    inline constexpr double kTranslationI = 0.0;
    inline constexpr double kTranslationD = 0.0;

    inline constexpr double kRotationP = 5.0;
    inline constexpr double kRotationI = 0.0;
    inline constexpr double kRotationD = 0.0;
}

namespace IntakeConstants {
    inline constexpr int kIntakeMotorPort = 10;
    inline constexpr int kIntakePivotPort = 9;
    //inline constexpr int kIntakeCANPort = 0;

    inline constexpr double kIntakeUp = 0.0;
    inline constexpr double kIntakeDown = 57.0;
    inline constexpr double kFFPivot = 0.0;
    inline constexpr units::volt_t kIntakeVolts = 5_V;

    inline constexpr double kSPivot = 0.0;      
    inline constexpr double kGPivot = 0.0;          
    inline constexpr double kVPivot = 0.0;          


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

    inline constexpr units::meters_per_second_squared_t kGravity = 9.80665_mps_sq;
    inline constexpr units::meter_t kShooterOffsetX = -0.2803_m;
    inline constexpr units::meter_t kShooterOffsetY = -0.2041_m;
    inline constexpr units::meter_t kShooterHeight = 0.525_m;

    inline constexpr units::revolutions_per_minute_t kMinRPM = 3200_rpm;
    inline constexpr units::revolutions_per_minute_t kMaxRPM = 4000_rpm;

    inline constexpr units::second_t kMaxFlightTime = 1.5_s;
    inline constexpr units::meter_t kFlywheelDiameter = 0.1016_m;

    //Hood
    inline constexpr int kHoodMotorPort = 0;   // TODO: real CAN ID

    inline constexpr units::degree_t kMinAngle = 20_deg;
    inline constexpr units::degree_t kMaxAngle = 60_deg;
    inline constexpr double kHoodUp = 20.0; 
    inline constexpr double kHoodDown = 0.0; 
    inline constexpr units::degree_t kAngleStep = 1_deg;
    inline constexpr units::revolutions_per_minute_t kMaxStep = 25_rpm;
    

    inline constexpr double kFFHood = 0.0;
    inline constexpr units::meter_t kRangeTolerance = 0.10_m;
    inline constexpr double kHoodGearRatio = 1.0;   // TODO: real motor-rotations-per-hood-rotation ratio
}


namespace RobotConstants{
    inline constexpr double kStartVoltage = 11;
}

namespace FieldConstants{
    inline constexpr units::meter_t kFieldLength = 8.0696_m;
    inline constexpr units::meter_t kfieldWidth = 7.1882_m;
    inline constexpr units::meter_t kTargetHeight = 1.8288_m;
    inline constexpr units::meter_t kBluePosX = 4.0284_m;
    inline constexpr units::meter_t kHubPosY = 3.5941_m;

    inline constexpr frc::Translation3d kBlueTargetPosition{kBluePosX, kHubPosY, kTargetHeight};
    inline constexpr frc::Translation3d kRedTargetPosition{kFieldLength - kBluePosX, kHubPosY, kTargetHeight};
}
