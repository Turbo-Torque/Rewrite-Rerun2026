#pragma once

#include <string>
#include "units/angle.h"
#include "units/angular_velocity.h"

namespace turbolib::structure {
struct ShooterSetpoint {
  std::string name;
  units::revolutions_per_minute_t rpm;
  units::degree_t angle;
};
}  // namespace turbolib::structure
