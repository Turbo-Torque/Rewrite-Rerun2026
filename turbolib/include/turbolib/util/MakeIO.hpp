#pragma once

#include <memory>
#include "frc/RobotBase.h"

namespace turbolib::utils {
template <typename BaseIO, typename RealIO, typename SimIO>
std::unique_ptr<BaseIO> MakeIO() {
  if (frc::RobotBase::IsReal()) {
    return std::make_unique<RealIO>();
  }
  return std::make_unique<SimIO>();
}
}  // namespace turbolib::utils