// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "frc/RobotBase.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "networktables/StructTopic.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/simulation/PhotonCameraSim.h"
#include "photon/simulation/VisionSystemSim.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

namespace turbolib::perception {
class TurboPhotonCamera {
private:
  frc::AprilTagFieldLayout layout;
  photon::PhotonCamera camera;
  photon::PhotonPoseEstimator poseEstimator;

  std::optional<photon::VisionSystemSim> systemSim = std::nullopt;
  std::optional<photon::PhotonCameraSim> cameraSim = std::nullopt;
  std::optional<photon::PhotonPipelineResult> lastResult = std::nullopt;

  nt::StructPublisher<frc::Pose2d> visionPosePublisher;

  bool seesTag = false;

public:
  TurboPhotonCamera(const std::string &cameraName,
                    const frc::Transform3d &cameraInBotSpace,
                    frc::AprilTagField field,
                    bool enableSim = frc::RobotBase::IsSimulation());

  void UpdateSim(const frc::Pose2d &robotPose);
  void UpdateHeading(const frc::Rotation2d &gyroAngle);
  void ResetHeading(const frc::Rotation2d &gyroAngle);

  std::vector<structure::PoseTimestampPair> FetchPose();
  std::optional<photon::PhotonPipelineResult> GetLastResult() const {
    return lastResult;
  }

  bool SeesTag() const { return seesTag; }
  bool IsSimulated() const { return cameraSim.has_value(); }
};
} // namespace turbolib::perception
