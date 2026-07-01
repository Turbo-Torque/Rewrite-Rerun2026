#pragma once

#include "frc2/command/SubsystemBase.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "ctre/phoenix6/CANBus.hpp"
#include "frc/controller/PIDController.h"
#include "turbolib/motors/NeoKrakenModule.hpp"
#include <frc2/command/CommandPtr.h>
#include <functional>
#include <array>



class DrivebaseSubsystem final:public frc2::SubsystemBase {
    public: 
    DrivebaseSubsystem();

    void Drive(const frc::ChassisSpeeds& speeds);
    void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states);

    std::array<frc::SwerveModuleState, 4> GetModuleStates();
    std::array<frc::SwerveModulePosition, 4> GetSwerveModulePosition();

    void Periodic() override;
    void SimulationPeriodic() override;

    frc2::CommandPtr DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> rotationSpeed);

    private:
    ctre::phoenix6::CANBus canBus{};
    turbolib::motors::NeoKrakenModule frontLeft, frontRight, backLeft, backRight;

    frc::PIDController rotationController{0.009, 0, 0};

    frc::ChassisSpeeds cmdSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s};

};