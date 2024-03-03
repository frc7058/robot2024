// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "constants/GeneralConstants.h"
#include "constants/DriveConstants.h"
#include "lib/Util.h"
#include "commands/IntakeCommands.h"

RobotContainer::RobotContainer() {
  ConfigureDriveControls();
  ConfigureShooterControls();
}

void RobotContainer::ConfigureDriveControls() 
{
  m_driveBase.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      double leftX = frc::ApplyDeadband(m_driveController.GetLeftX(), constants::controls::joystickDeadband);
      double leftY = frc::ApplyDeadband(m_driveController.GetLeftY(), constants::controls::joystickDeadband);
      double rightX = frc::ApplyDeadband(m_driveController.GetRightX(), constants::controls::joystickDeadband);

      units::meters_per_second_t velocityX = util::sign(leftY) * -(leftY * leftY) * constants::drive::maxDriveVelocity;
      units::meters_per_second_t velocityY = util::sign(leftX) * -(leftX * leftX) * constants::drive::maxDriveVelocity;
      units::radians_per_second_t angularVelocity = util::sign(rightX) * -(rightX * rightX) * constants::drive::maxAngularVelocity;

      m_driveBase.Drive(velocityX, velocityY, angularVelocity, true);
    },
    {&m_driveBase}
  ));
}

void RobotContainer::ConfigureShooterControls()
{
  // Run intake while A is held
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kA)
    .WhileTrue(IntakeCommands::RunIntake(&m_intake));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto("name").ToPtr();
  //return frc2::cmd::Print("No autonomous command configured");
}
