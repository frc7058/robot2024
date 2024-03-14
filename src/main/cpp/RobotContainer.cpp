// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "commands/DriveCommand.h"
#include "commands/IntakeCommands.h"
#include "commands/ShooterCommands.h"

RobotContainer::RobotContainer() {
  ConfigureDriveControls();
  ConfigureShooterControls();
}

void RobotContainer::ConfigureDriveControls() 
{
  m_driveBase.SetDefaultCommand(DriveCommand(&m_driveBase, m_vision, m_driveController));

  frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kX)
    .OnTrue(frc2::cmd::RunOnce([this] { m_driveBase.ZeroHeading(); }, {}));
}

void RobotContainer::ConfigureShooterControls()
{
  // Run intake while A is held
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kA)
    .WhileTrue(IntakeCommands::RunIntake(&m_intake));

  /*
  // Uncomment to test just the shooter wheels
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kRightBumper)
    .OnTrue(ShooterCommands::RunShooterWheels(&m_shooter, 600_rpm))
    .OnFalse(ShooterCommands::StopShooterWheels(&m_shooter));
  */

  // Compound shoot command. Runs shooter wheels and intake at the same time
  // frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kY)
  //   .OnTrue(ShooterCommands::ShootTest(&m_shooter, &m_intake));

    
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kB)
    .OnTrue(IntakeCommands::EjectIntake(&m_intake))
    .OnFalse(IntakeCommands::StopIntake(&m_intake));

  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kY)
    .OnTrue(ShooterCommands::RunShooterWheelsConstantVoltage(&m_shooter, 12.0_V))
    .OnFalse(ShooterCommands::StopShooterWheels(&m_shooter));

  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kX)
    .OnTrue(ShooterCommands::RunFeeder(&m_shooter))
    .OnFalse(ShooterCommands::StopFeeder(&m_shooter));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto("rotate").ToPtr()
    .OnlyWhile([&] { return m_driveBase.IsNavXAvailable(); })
    .FinallyDo([&] { m_driveBase.Stop(); });
}

void RobotContainer::InitSysId()
{
}