#include "subsystems/ShotSolve.h"
#include "subsystems/DrivebaseSubsystem.h"

#include "Constants.h"
#include <frc/DriverStation.h>
#include <cmath>
#include <units/math.h>
#include <algorithm>
#include "frc/smartdashboard/SmartDashboard.h"

ShotSolve::ShotGeometry ShotSolve::ComputeGeometry(const frc::Translation3d& muzzle, const frc::Translation3d& target) {
    units::meter_t dx = target.X() - muzzle.X();
    units::meter_t dy = target.Y() - muzzle.Y();
    units::meter_t dz = target.Z() - muzzle.Z();
    frc::Translation2d groundOffset{dx, dy};

    return ShotGeometry{groundOffset.Norm(), dz, groundOffset.Angle()};
};

frc::Translation3d ShotSolve::GetMuzzlePosition(const frc::Pose2d& robotPose) {
    frc::Translation2d offset{ShooterConstants::kShooterOffsetX, ShooterConstants::kShooterOffsetY};
    frc::Translation2d fieldOffset = offset.RotateBy(robotPose.Rotation());
    frc::Translation2d muzzleXY = robotPose.Translation() + fieldOffset;

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

        for (const auto& candidate : candidates) {
            units::radians_per_second_t omega{candidate.rpm};
            units::meters_per_second_t exitSpeed{radius.value() * omega.value()};

            double cosAngle = units::math::cos(candidate.hoodAngle);
            double sinAngle = units::math::sin(candidate.hoodAngle);

            units::meters_per_second_t vHorizontal = exitSpeed * cosAngle;
            units::meters_per_second_t vVertical = exitSpeed * sinAngle;

            double a = 0.5 * ShooterConstants::kGravity.value();
            double b = -vVertical.value();
            double c = geometry.heightDelta.value();
            double discriminant = b * b - 4 * a * c;

            ShotEvaluation evaluation{candidate, 0_m, 0_s, false};

            if (discriminant >= 0) {
                double t1 = (-b - std::sqrt(discriminant)) / ( 2 * a);
                double t2 = (-b + std::sqrt(discriminant)) / ( 2 * a);
                double t = std::max(t1, t2);

                if ( t > 0 ) {
                    evaluation.reachedHeight = true;
                    evaluation.flightTime = units::second_t{t};

                    evaluation.achievedRange = vHorizontal * evaluation.flightTime;
                }
            }
            evaluations.push_back(evaluation);
        }
        return evaluations;
    }

std::vector<ShotSolve::ShotEvaluation> ShotSolve::FilterValidShots (const std::vector<ShotEvaluation>& evaluations, const ShotGeometry& geometry) {
    std::vector<ShotEvaluation> validShots;

    for (const auto& evaluation : evaluations) {
        if (!evaluation.reachedHeight) continue;
        if (evaluation.flightTime > ShooterConstants::kMaxFlightTime) continue;
        
        units::meter_t rangeError = units::math::abs(evaluation.achievedRange - geometry.groundDistance);
        if (rangeError > ShooterConstants::kRangeTolerance) continue;

        validShots.push_back(evaluation);
    }
    return validShots;
}

void ShotSolve::RankShots(std::vector<ShotEvaluation>& validShots) {
    std::sort(validShots.begin(), validShots.end(),
        [](const ShotEvaluation& a, const ShotEvaluation& b) {
            return a.flightTime < b.flightTime;
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
    bool isRed = alliance == frc::DriverStation::Alliance::kRed;

    frc::SmartDashboard::PutBoolean("HasAlliance", alliance.has_value());
    frc::SmartDashboard::PutBoolean("IsRed", isRed);

    if (isRed) {
        return FieldConstants::kRedTargetPosition;
    }
    return FieldConstants::kBlueTargetPosition;
}
