#pragma once
#include "units/voltage.h"
#include "units/current.h"
#include "units/angular_velocity.h"



struct ShooterIOInputs{
    units::revolutions_per_minute_t shooterRPM{0.0};
    units::revolutions_per_minute_t shooterRPMSsetpoint{0.0};
    units::ampere_t shooterCurrent{0_A};

    bool atRotations = false;

};

class ShooterIO {
    public:
        virtual void UpdateInputs(ShooterIOInputs& inputs) = 0;
        virtual void SetShooterRPM(units::revolutions_per_minute_t rpm) = 0;
        virtual ~ShooterIO() = default;
};