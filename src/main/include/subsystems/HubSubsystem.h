
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <string>
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
    matchPhasePublisher.Set(ToString(GetMatchPhase()));
    allianceActivePublisher.Set(IsAllianceActive(frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kRed)));
    frc::SmartDashboard::PutNumber("TimeUntilPhaseChange", GetTimeUntilPhaseChange().value());

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

    if (time > 140_s) {
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

    if (time > 140_s) {
      return time - 140_s;
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

  bool IsAllianceActive(frc::DriverStation::Alliance alliance) const {
    if (inactiveFirst == 'U') {
      return true;  // Assume both are active until we know otherwise
    }

    MatchPhase phase = GetMatchPhase();

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

 private:
  char inactiveFirst{'U'};

  nt::StringPublisher inactiveFirstPublisher =
      nt::NetworkTableInstance::GetDefault().GetStringTopic("HubSubsystem/InactiveFirst").Publish();
  nt::StringPublisher matchPhasePublisher =
      nt::NetworkTableInstance::GetDefault().GetStringTopic("HubSubsystem/MatchPhase").Publish();
  nt::BooleanPublisher allianceActivePublisher =
      nt::NetworkTableInstance::GetDefault().GetBooleanTopic("HubSubsystem/IsAllianceActive").Publish();

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