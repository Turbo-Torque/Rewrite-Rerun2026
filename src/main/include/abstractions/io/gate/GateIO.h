#pragma once
#include "units/voltage.h"
#include "units/current.h"


struct GateIOInputs{
    units::volt_t gateVolts{0_V};
    units::ampere_t gateCurrent{0_A};

};

class GateIO {
    public:
        virtual void UpdateInputs(GateIOInputs& inputs) = 0;
        virtual void SetGateVolts(units::volt_t volts) = 0;
        virtual ~GateIO() = default;
};