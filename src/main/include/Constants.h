

#pragma once



namespace OperatorConstants {

    inline constexpr int kDriverControllerPort = 0;

}

namespace DriveConstants {
    constexpr int kFrontLeftDriveID = 1;
    constexpr int kFrontLeftTurnID = 2;
    constexpr int kFrontLeftEncoderID = 9;

    constexpr int kFrontRightDriveID = 3;
    constexpr int kFrontRightTurnID = 4;
    constexpr int kFrontRightEncoderID = 10;


    constexpr int kBackLeftDriveID = 5;
    constexpr int kBackLeftTurnID = 6;
    constexpr int kBackLeftEncoderID = 11;


    constexpr int kBackRightDriveID = 7;
    constexpr int kBackRightTurnID = 8;
    constexpr int kBackRightEncoderID = 12;


    constexpr double kTrackWidth = 0.381;
    constexpr double kWheelBase = 0.381;

    constexpr double kMaxSpeed = 4.5;
}
