

#pragma once

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include <turbolib/structure/SwervePorts.hpp>



namespace OperatorConstants {

    inline constexpr int kDriverControllerPort = 0;

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


    inline constexpr double kMaxSpeed = 4.5;

}
