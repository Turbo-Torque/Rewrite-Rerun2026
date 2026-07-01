// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "turbolib/perception/TurboPoseEstimator.hpp"

#include <cassert>
#include <vector>

#include "frc/DataLogManager.h"
#include "frc/geometry/Pose2d.h"
#include "turbolib/structure/PoseTimestampPair.hpp"

using namespace turbolib::perception;

frc::Pose2d TurboPoseEstimator::GetPose2D() {
  return poseEstimator.GetEstimatedPosition();
}

void TurboPoseEstimator::ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                                                const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                                                const frc::Pose2d& pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);

  for (const auto& camera : localizationCameras) {
    camera->ResetHeading(gyroAngle);
  }
}

void TurboPoseEstimator::UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                                     const std::array<frc::SwerveModulePosition, 4>& modulePositions) {
  poseEstimator.Update(gyroAngle, modulePositions);

  if (visionEnabled) {
    UpdateWithAllAvailableVisionMeasurements(gyroAngle);
  }
}

void TurboPoseEstimator::TryVisionUpdateWithCamera(turbolib::perception::TurboPhotonCamera& camera,
                                                   const frc::Rotation2d& gyroAngle) {
  const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera.FetchPose();

  for (const auto& pair : visionPoses) {
    poseEstimator.AddVisionMeasurement(pair.getPose(), pair.getLatency());
  }
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements(const frc::Rotation2d& gyroAngle) {
  if (localizationCameras.empty()) {
    frc::DataLogManager::Log("No localization cameras configured for pose estimation!");
    return;
  }

  for (auto& camera : localizationCameras) {
    TryVisionUpdateWithCamera(*camera, gyroAngle);
  }
}

void TurboPoseEstimator::UpdateAllSims(frc::Pose2d pose) {
  for (auto& camera : localizationCameras) {
    camera->UpdateSim(pose);
  }
}

bool TurboPoseEstimator::SeesTag() const {
  return std::any_of(localizationCameras.begin(), localizationCameras.end(),
                     [](const std::unique_ptr<TurboPhotonCamera>& cam) { return cam->SeesTag(); });
}
