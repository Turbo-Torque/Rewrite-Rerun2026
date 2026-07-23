#pragma once
#include "units/voltage.h"
#include "units/current.h"
#include "units/angular_velocity.h"



struct ShooterIOInputs{
    units::revolutions_per_minute_t shooterRPM{0.0};
    units::revolutions_per_minute_t shooterRPMsetpoint{0.0};
    units::ampere_t shooterCurrent{0_A};
    units::ampere_t hoodCurrent{0_A};
    units::volt_t hoodPID{0_V};
    units::volt_t hoodFF{0_V};
    double hoodPosition{0.0};
    double hoodSetPoint{0.0};


    bool atRotations = false;
    bool hoodAtSetpoint{false};

};

class ShooterIO {
    public:
        virtual void UpdateInputs(ShooterIOInputs& inputs) = 0;
        virtual void SetShooterRPM(units::revolutions_per_minute_t rpm) = 0;
        virtual void SetHoodSetpoint(double rot) = 0;
        virtual ~ShooterIO() = default;
};