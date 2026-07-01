

#pragma once

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include <turbolib/structure/SwervePorts.hpp>



namespace OperatorConstants {

    inline constexpr int kDriveControllerPort = 0;

}

namespace DriveConstants {
    inline turbolib::structure::SwervePorts kFrontLeftPorts{1, 2, 18};
    inline turbolib::structure::SwervePorts kFrontRightPorts{3, 8, 21};
    inline turbolib::structure::SwervePorts kBackLeftPorts{5, 4, 19};
    inline turbolib::structure::SwervePorts kBackRightPorts{7, 6, 20};

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
}

namespace IntakeConstants {
    inline constexpr int kIntakeMotorPort = 0;
    inline constexpr int kIntakePivotPort = 0;
    inline constexpr int kIntakeCANPort = 0;

    inline constexpr double kIntakeUp = 0.0;
    inline constexpr double kIntakeDown = 90.0;
    inline constexpr double kFFPivot = 0.0;
    inline constexpr units::volt_t kIntakeVolts = 3_V;

}
