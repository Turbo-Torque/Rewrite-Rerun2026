// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "frc/DriverStation.h"
#include "frc/DataLogManager.h"
#include "frc/smartdashboard/SmartDashboard.h"

Robot::Robot() {
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  frc::SmartDashboard::PutData("Command Scheduler", &frc2::CommandScheduler::GetInstance());

}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();


  frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }

  // frc2::CommandScheduler::GetInstance().Schedule(m_container.GetClimbSubsystem().SetClimbSetpointCommand(0.0));
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
