#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "units/angular_velocity.h"
#include <vector>
#include <optional>

namespace ShotSolve{
    struct ShotGeometry {
        units::meter_t groundDistance;
        units::meter_t heightDelta;
        frc::Rotation2d bearing;
    };

    struct ShotCandidate {
        units::revolutions_per_minute_t rpm;
        units::degree_t hoodAngle;
    };

    struct ShotEvaluation {
        ShotCandidate candidate;
        units::meter_t achievedRange;
        units::second_t flightTime;
        bool reachedHeight;
    };


    frc::Translation3d GetMuzzlePosition(const frc::Pose2d& robotPose);
    frc::Translation3d GetTargetPosition();
    ShotGeometry ComputeGeometry(const frc::Translation3d& muzzle, const frc::Translation3d& target);

    std::vector<ShotCandidate> GenerateCandidates();
    std::vector<ShotEvaluation> EvaluateCandidates(const std::vector<ShotCandidate>& candidates, const ShotGeometry& geometry);
    std::vector<ShotEvaluation> FilterValidShots(const std::vector<ShotEvaluation>& evaluations, const ShotGeometry& geometry);

    void RankShots(std::vector<ShotEvaluation>& validShots);

    std::optional<ShotEvaluation> SelectBestShot(const std::vector<ShotEvaluation>& rankedShots);
}