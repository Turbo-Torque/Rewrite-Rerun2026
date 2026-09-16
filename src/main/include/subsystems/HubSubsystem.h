
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <array>
#include <string>
#include "frc/Alert.h"
#include "frc/DriverStation.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/BooleanTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include "units/time.h"

enum class MatchPhase { Auto, Transition, Shift1, Shift2, Shift3, Shift4, EndGame, Ended };

class HubSubsystem final : public frc2::SubsystemBase {
 public:
  HubSubsystem() = default;

  void Periodic() override {
    auto myAlliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kRed);
    auto opponentAlliance =
        myAlliance == frc::DriverStation::Alliance::kRed ? frc::DriverStation::Alliance::kBlue : frc::DriverStation::Alliance::kRed;

    bool myHubActive = IsAllianceActive(myAlliance);

    matchPhasePublisher.Set(ToString(GetMatchPhase()));
    allianceActivePublisher.Set(myHubActive);
    opponentActivePublisher.Set(IsAllianceActive(opponentAlliance));
    frc::SmartDashboard::PutNumber("TimeUntilPhaseChange", GetTimeUntilPhaseChange().value());
    // ADDED: actual countdown to the next Active<->Inactive flip, not just the next phase boundary
    frc::SmartDashboard::PutNumber("TimeUntilStatusChange", GetTimeUntilStatusChange(myAlliance).value());

    // Driver-visible warning the moment our own hub goes inactive — shows on the Driver
    // Station and Elastic's Alerts widget automatically, no extra dashboard config needed.
    hubInactiveAlert.Set(!myHubActive);

    if (inactiveFirst != 'U') {
      return;
    }

    std::string gameData = frc::DriverStation::GetGameSpecificMessage();

    if (!gameData.empty()) {
      inactiveFirst = gameData[0];

      switch (inactiveFirst) {
        case 'R':
          inactiveFirstPublisher.Set("Red");
          break;
        case 'B':
          inactiveFirstPublisher.Set("Blue");
          break;
        default:
          inactiveFirstPublisher.Set("Unknown");
          break;
      }
    }
  }

  MatchPhase GetMatchPhase() const {
    auto time = frc::DriverStation::GetMatchTime();

    if (time == -1_s) {
      return MatchPhase::Ended;
    }

    // FIXED: GetMatchTime() resets and counts down independently each period — during real
    // autonomous it counts down from the ~20s auto period, never from a value above 140s. The
    // old `time > 140_s` check could never be true during actual auto, and auto's real 0-20s
    // readings fell into the same range as EndGame below, misreporting Auto as EndGame.
    if (frc::DriverStation::IsAutonomous()) {
      return MatchPhase::Auto;
    } else if (time > 130_s) {
      return MatchPhase::Transition;
    } else if (time > 105_s) {
      return MatchPhase::Shift1;
    } else if (time > 80_s) {
      return MatchPhase::Shift2;
    } else if (time > 55_s) {
      return MatchPhase::Shift3;
    } else if (time > 30_s) {
      return MatchPhase::Shift4;
    } else {
      return MatchPhase::EndGame;
    }
  }

  units::second_t GetTimeUntilPhaseChange() {
    auto time = frc::DriverStation::GetMatchTime();

    if (time == -1_s) {
      return 0_s;
    }

    if (frc::DriverStation::IsAutonomous()) {
      return time;
    } else if (time > 130_s) {
      return time - 130_s;
    } else if (time > 105_s) {
      return time - 105_s;
    } else if (time > 80_s) {
      return time - 80_s;
    } else if (time > 55_s) {
      return time - 55_s;
    } else if (time > 30_s) {
      return time - 30_s;
    } else {
      return time;
    }
  }

  char GetInactiveFirst() const { return inactiveFirst; }

  bool IsAllianceActive(frc::DriverStation::Alliance alliance) const { return IsActiveAtPhase(alliance, GetMatchPhase()); }

  // ADDED: time until MY active/inactive status actually flips — not just "time until the next
  // phase," since not every phase boundary flips it (Auto->Transition never flips; whether
  // Shift4->Endgame flips depends on which alliance you are). Walks the known boundaries forward
  // and returns how long until the first one where status actually differs from right now.
  units::second_t GetTimeUntilStatusChange(frc::DriverStation::Alliance alliance) const {
    auto time = frc::DriverStation::GetMatchTime();
    if (time == -1_s) {
      return 0_s;
    }

    if (frc::DriverStation::IsAutonomous()) {
      // Both alliances are always Active through Auto and the fixed 10s Transition that follows
      // it, so nothing flips until Shift1 begins — auto's own clock plus the known transition length.
      return time + 10_s;
    }

    bool currentStatus = IsActiveAtPhase(alliance, GetMatchPhase());

    for (const auto& boundary : kUpcomingPhaseBoundaries) {
      if (boundary.endTime >= time) {
        continue;  // already reached/passed this boundary
      }
      bool statusThere = IsActiveAtPhase(alliance, boundary.phase);
      if (statusThere != currentStatus) {
        return time - boundary.endTime;
      }
      currentStatus = statusThere;
    }

    return time;  // no further flip before match end
  }

 private:
  char inactiveFirst{'U'};

  // ADDED: driver-visible alert + opponent hub status, for Elastic
  frc::Alert hubInactiveAlert{"Hub Inactive", frc::Alert::AlertType::kWarning};

  // ADDED: pure phase->status lookup, extracted out of IsAllianceActive so GetTimeUntilStatusChange
  // can evaluate status at a hypothetical future phase without needing the real match clock to be there yet.
  bool IsActiveAtPhase(frc::DriverStation::Alliance alliance, MatchPhase phase) const {
    if (inactiveFirst == 'U') {
      return true;  // Assume both are active until we know otherwise
    }

    if (phase == MatchPhase::Transition || phase == MatchPhase::EndGame || phase == MatchPhase::Auto) {
      return true;
    }

    if (inactiveFirst == 'R') {
      return (phase == MatchPhase::Shift1 || phase == MatchPhase::Shift3)
                 ? alliance == frc::DriverStation::Alliance::kBlue
                 : alliance == frc::DriverStation::Alliance::kRed;
    } else if (inactiveFirst == 'B') {
      return (phase == MatchPhase::Shift1 || phase == MatchPhase::Shift3)
                 ? alliance == frc::DriverStation::Alliance::kRed
                 : alliance == frc::DriverStation::Alliance::kBlue;
    } else {
      return true;
    }
  }

  struct PhaseBoundary {
    MatchPhase phase;
    units::second_t endTime;
  };

  // The teleop-clock value at which each phase BEGINS (matches GetMatchPhase()'s thresholds),
  // in chronological order. Auto is handled separately in GetTimeUntilStatusChange.
  static constexpr std::array<PhaseBoundary, 6> kUpcomingPhaseBoundaries{{
      {MatchPhase::Shift1, 130_s},
      {MatchPhase::Shift2, 105_s},
      {MatchPhase::Shift3, 80_s},
      {MatchPhase::Shift4, 55_s},
      {MatchPhase::EndGame, 30_s},
      {MatchPhase::Ended, 0_s},
  }};

  nt::StringPublisher inactiveFirstPublisher =
      nt::NetworkTableInstance::GetDefault().GetStringTopic("HubSubsystem/InactiveFirst").Publish();
  nt::StringPublisher matchPhasePublisher =
      nt::NetworkTableInstance::GetDefault().GetStringTopic("HubSubsystem/MatchPhase").Publish();
  nt::BooleanPublisher allianceActivePublisher =
      nt::NetworkTableInstance::GetDefault().GetBooleanTopic("HubSubsystem/IsAllianceActive").Publish();
  nt::BooleanPublisher opponentActivePublisher =
      nt::NetworkTableInstance::GetDefault().GetBooleanTopic("HubSubsystem/IsOpponentActive").Publish();

  std::string ToString(MatchPhase phase) const {
    switch (phase) {
      case MatchPhase::Auto:
        return "Auto";
      case MatchPhase::Transition:
        return "Transition";
      case MatchPhase::Shift1:
        return "Shift 1";
      case MatchPhase::Shift2:
        return "Shift 2";
      case MatchPhase::Shift3:
        return "Shift 3";
      case MatchPhase::Shift4:
        return "Shift 4";
      case MatchPhase::EndGame:
        return "Endgame";
      default:
        return "Unknown";
    }
  }
};