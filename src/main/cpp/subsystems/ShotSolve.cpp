#include "subsystems/ShotSolve.h"
#include "subsystems/DrivebaseSubsystem.h"

#include "Constants.h"
#include <cstdlib>
#include <frc/DriverStation.h>
#include <cmath>
#include <units/math.h>
#include <algorithm>
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/length.h"

ShotSolve::ShotGeometry ShotSolve::ComputeGeometry(const frc::Translation3d& muzzle, const frc::Translation3d& target) {
    units::meter_t dx = target.X() - muzzle.X();
    units::meter_t dy = target.Y() - muzzle.Y();
    frc::Translation2d groundOffset{dx, dy};

    // FIXED: these were both writing to the same NT key ("distance"), so the angle PutNumber was
    // silently overwriting the distance one — the dashboard was only ever showing the angle.
    frc::SmartDashboard::PutNumber("distance", groundOffset.Norm().value());
    frc::SmartDashboard::PutNumber("bearing", groundOffset.Angle().Degrees().value());

    return ShotGeometry{groundOffset.Norm(), groundOffset.Angle()};
};

frc::Translation3d ShotSolve::GetMuzzlePosition(const frc::Pose2d& robotPose) {
    frc::Translation2d offset{ShooterConstants::kShooterOffsetX, ShooterConstants::kShooterOffsetY};
    frc::Translation2d fieldOffset = offset.RotateBy(robotPose.Rotation());
    // frc::Translation2d muzzleXY = robotPose.Translation() + fieldOffset;
    frc::Translation2d muzzleXY = robotPose.Translation();

    return frc::Translation3d{muzzleXY.X(), muzzleXY.Y(), ShooterConstants::kShooterHeight};
};

std::vector<ShotSolve::ShotCandidate> ShotSolve::GenerateCandidates() {
    std::vector<ShotCandidate> candidates;

    for (auto rpm = ShooterConstants::kMinRPM; rpm <= ShooterConstants::kMaxRPM; rpm += ShooterConstants::kMaxStep) {
        for (auto angle = ShooterConstants::kMinAngle; angle <= ShooterConstants::kMaxAngle; angle += ShooterConstants::kAngleStep) {
            candidates.push_back(ShotCandidate{rpm, angle});
        }
    }
    return candidates;
}

std::vector<ShotSolve::ShotEvaluation> ShotSolve::EvaluateCandidates(
    const std::vector<ShotCandidate>& candidates, const ShotGeometry& geometry) {
        std::vector<ShotEvaluation> evaluations;
        units::meter_t radius = ShooterConstants::kFlywheelDiameter / 2;

        // FIXED: efficiency was live-tunable on the dashboard during testing — locked back in as a
        // constant now that 0.435 is confirmed good. Obstacle offset is still live-tunable.
        double efficiency = ShooterConstants::kFlywheelEfficiency;
        units::meter_t obstacleDistanceFromTarget{frc::SmartDashboard::GetNumber("Obstacle Distance From Target", ShooterConstants::kObstacleDistanceFromTarget.value())};

        double a = 0.5 * ShooterConstants::kGravity.value();
        double requiredObstacleHeight = (ShooterConstants::kObstacleHeight - ShooterConstants::kShooterHeight).value();
        // ADDED: the hex is symmetric enough around the target to treat the far wall as the same
        // distance past the target as the near wall/obstacle is before it. "Reaches far side" just
        // means still airborne (hasn't hit the floor) at that point — no separate exact-height check.
        double floorLevelAboveMuzzle = -ShooterConstants::kShooterHeight.value();

        for (const auto& candidate : candidates) {
            units::radians_per_second_t omega{candidate.rpm};
            // FIXED: real ball exit speed is less than theoretical flywheel surface speed
            // (slip/compression) — efficiency is calibrated from real measured shots.
            units::meters_per_second_t exitSpeed{efficiency * radius.value() * omega.value()};

            // FIXED: hoodAngle (kMinAngle..kMaxAngle) is measured from VERTICAL (straight up), not
            // from horizontal — a smaller angle means closer to straight up (steeper climb), a larger
            // angle means closer to flat. So the vertical component uses cos and the horizontal
            // component uses sin (the opposite of the usual "angle from horizontal" convention).
            double cosAngle = units::math::cos(candidate.hoodAngle);
            double sinAngle = units::math::sin(candidate.hoodAngle);

            double vH = exitSpeed.value() * sinAngle;
            double vV = exitSpeed.value() * cosAngle;

            ShotEvaluation evaluation{candidate, 0_s, false, false};

            if (vH > 0.0) {
                double tTarget = geometry.groundDistance.value() / vH;
                evaluation.flightTime = units::second_t{tTarget};

                units::meter_t obstacleDistance = geometry.groundDistance - obstacleDistanceFromTarget;
                if (obstacleDistance > 0_m) {
                    double tObstacle = obstacleDistance.value() / vH;
                    double heightAtObstacle = vV * tObstacle - a * tObstacle * tObstacle;
                    evaluation.clearsObstacle = heightAtObstacle >= requiredObstacleHeight;
                }

                units::meter_t farSideDistance = geometry.groundDistance + obstacleDistanceFromTarget;
                double tFarSide = farSideDistance.value() / vH;
                double heightAtFarSide = vV * tFarSide - a * tFarSide * tFarSide;
                evaluation.reachesFarSide = heightAtFarSide >= floorLevelAboveMuzzle;
            }
            evaluations.push_back(evaluation);
        }
        return evaluations;
    }

std::vector<ShotSolve::ShotEvaluation> ShotSolve::FilterValidShots (const std::vector<ShotEvaluation>& evaluations, const ShotGeometry& geometry) {
    std::vector<ShotEvaluation> validShots;

    for (const auto& evaluation : evaluations) {
        if (!evaluation.clearsObstacle) continue;   // must clear the rim on the way in
        if (!evaluation.reachesFarSide) continue;   // must still be airborne once across the hex
        if (evaluation.flightTime > ShooterConstants::kMaxFlightTime) continue;

        validShots.push_back(evaluation);
    }
    return validShots;
}

void ShotSolve::RankShots(std::vector<ShotEvaluation>& validShots) {
    // FIXED: was ranking by shortest flight time, which structurally always favors max RPM (more
    // speed = less flight time = always wins, and also makes clearing the obstacle easier) — angle
    // was the only thing that ever actually varied. Rank by lowest RPM first (gentlest shot that
    // still makes it), then by lowest angle as a tiebreak among candidates at the same RPM.
    std::sort(validShots.begin(), validShots.end(),
        [](const ShotEvaluation& a, const ShotEvaluation& b) {
            if (a.candidate.rpm != b.candidate.rpm) {
                return a.candidate.rpm < b.candidate.rpm;
            }
            return a.candidate.hoodAngle < b.candidate.hoodAngle;
        });
}

std::optional<ShotSolve::ShotEvaluation> ShotSolve::SelectBestShot(const std::vector<ShotEvaluation>& rankedShots) {
    if (rankedShots.empty()) {
        return std::nullopt;
    }
    return rankedShots.front();
}

frc::Translation3d ShotSolve::GetTargetPosition() {
    auto alliance = frc::DriverStation::GetAlliance();
    bool isRed = alliance.value_or(frc::DriverStation::kRed) == frc::DriverStation::Alliance::kRed;

    frc::SmartDashboard::PutBoolean("HasAlliance", alliance.has_value());
    frc::SmartDashboard::PutBoolean("IsRed", isRed);

    if (isRed) {
        return FieldConstants::kRedTargetPosition;
    }
    return FieldConstants::kBlueTargetPosition;
}
