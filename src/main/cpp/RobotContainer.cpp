#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {

    ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {

    // Add controller bindings here

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

    return frc2::cmd::None();

}