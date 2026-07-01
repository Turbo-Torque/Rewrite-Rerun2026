#pragma once

#include "units/voltage.h"

struct HopperIOInputs {
    units::volt_t hopperVolts{0_V};
};

class HopperIO {
    public:
        virtual void UpdateInputs(HopperIOInputs& inputs) = 0;
        virtual void SetHopperVolts(units::volt_t volts) = 0;
        virtual ~HopperIO() = default;
};