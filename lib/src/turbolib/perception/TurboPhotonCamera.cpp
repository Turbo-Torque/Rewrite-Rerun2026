// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/perception/TurboPhotonCamera.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "frc/Timer.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

using namespace turbolib::perception;

TurboPhotonCamera::TurboPhotonCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace,
                                     frc::AprilTagField field, bool enableSim)
    : layout(frc::AprilTagFieldLayout::LoadField(field)), camera(cameraName), poseEstimator(layout, cameraInBotSpace) {
  visionPosePublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("DriveSubsystem/VisionPose").Publish();

  if (enableSim) {
    auto cameraProp = photon::SimCameraProperties();
    cameraProp.SetCalibration(640, 480, 75_deg);
    cameraProp.SetCalibError(0.25, 0.08);
    cameraProp.SetFPS(units::hertz_t{20});
    cameraProp.SetAvgLatency(20_ms);
    cameraProp.SetLatencyStdDev(3_ms);

    systemSim.emplace("main");
    cameraSim.emplace(photon::PhotonCameraSim(&camera, cameraProp));

    cameraSim->EnableRawStream(true);
    cameraSim->EnabledProcessedStream(true);
    cameraSim->EnableDrawWireframe(false);

    systemSim->AddAprilTags(layout);
    systemSim->AddCamera(&cameraSim.value(), cameraInBotSpace);

    frc::SmartDashboard::PutData("Sim Field", &systemSim->GetDebugField());
  }
}

void TurboPhotonCamera::UpdateSim(const frc::Pose2d& robotPose) {
  if (systemSim.has_value()) {
    systemSim->Update(robotPose);
  }
}

void TurboPhotonCamera::UpdateHeading(const frc::Rotation2d& gyroAngle) {
  poseEstimator.AddHeadingData(frc::Timer::GetFPGATimestamp(), gyroAngle);
}

void TurboPhotonCamera::ResetHeading(const frc::Rotation2d& gyroAngle) {
  poseEstimator.ResetHeadingData(frc::Timer::GetFPGATimestamp(), gyroAngle);
}

std::vector<turbolib::structure::PoseTimestampPair> TurboPhotonCamera::FetchPose() {
  const auto& results = camera.GetAllUnreadResults();
  std::vector<turbolib::structure::PoseTimestampPair> poses;
  poses.reserve(results.size());

  for (const auto& result : results) {
    lastResult = result;

    const auto target = result.GetBestTarget();

    if (target.GetPoseAmbiguity() > 0.2) {
      continue;
    }

    if (auto visionEst = poseEstimator.EstimateCoprocMultiTagPose(result)) {
      poses.emplace_back(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp);
    }
  }

  if (poses.empty()) {
    seesTag = false;
  } else {
    seesTag = true;
    visionPosePublisher.Set(poses.front().getPose());
  }

  return poses;
}
