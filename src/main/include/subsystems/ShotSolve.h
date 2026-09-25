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
        frc::Rotation2d bearing;
    };

    struct ShotCandidate {
        units::revolutions_per_minute_t rpm;
        units::degree_t hoodAngle;
    };

    struct ShotEvaluation {
        ShotCandidate candidate;
        units::second_t flightTime;
        // ADDED: clears the near wall of the hex (the obstacle/rim) AND is still airborne by the time
        // it's crossed to the hex's far side — once both hold, it drops into the scoring opening on
        // its own, so there's no separate exact-height-at-target check.
        bool clearsObstacle;
        bool reachesFarSide;
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