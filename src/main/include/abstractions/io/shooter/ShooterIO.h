#pragma once
#include "units/voltage.h"
#include "units/current.h"
#include "units/angular_velocity.h"



struct ShooterIOInputs{
    units::revolutions_per_minute_t shooterRPM{0_rpm};
    units::ampere_t shooterCurrent{0_A};

    bool atRotations{false};

};

class ShooterIO {
    public:
        virtual void UpdateInputs(ShooterIOInputs& inputs) = 0;
        virtual void SetShooterRPM(units::turns_per_second_t rpm) = 0;
        virtual ~ShooterIO() = default;
};