// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>
#include "constants/GeneralConstants.h"

void Robot::RobotInit() 
{
  if(constants::enableSysId)
  {
    m_container.InitSysId();
  }
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
  // std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

  // if(alliance)
  // {
  //   m_container.ConfigurePathPlanner();
  // }
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_container.SetDriveControlMode(ControlMode::ClosedLoop);

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) 
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() 
{
  m_container.SetDriveControlMode(ControlMode::OpenLoop);

  if (m_autonomousCommand) 
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
