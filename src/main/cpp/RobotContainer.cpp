// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "commands/DriveCommand.h"
#include "commands/IntakeCommands.h"
#include "commands/ShooterCommands.h"
#include "commands/ClimberCommands.h"

#include "constants/GeneralConstants.h"
#include "constants/ShooterConstants.h"
#include "constants/DriveConstants.h"
#include "constants/AutoConstants.h"
#include "lib/Util.h"
#include "lib/FieldUtil.h"

RobotContainer::RobotContainer() {
  if(!constants::enableSysId)
  {
    ConfigureDriveControls();
    ConfigureShooterControls();
  }

  m_autoChooser.SetDefaultOption(constants::autonomous::defaultAuto, constants::autonomous::defaultAuto);
  m_autoChooser.AddOption(constants::autonomous::oneNoteAuto, constants::autonomous::oneNoteAuto);

  for(const std::string& autoName : constants::autonomous::autoNames)
  {
    m_autoChooser.AddOption(autoName, autoName);
  }

  frc::SmartDashboard::PutData(&m_autoChooser);

  pathplanner::NamedCommands::registerCommand("RunIntake", IntakeCommands::RunIntake(&m_intake).WithTimeout(constants::autonomous::intakeTimeLimit));
  pathplanner::NamedCommands::registerCommand("ShootToSpeaker", ShooterCommands::ShootToSpeaker(&m_shooter, &m_intake));
  // pathplanner::NamedCommands::registerCommand("RunShooterWheels", ShooterCommands::RunShooterWheels(&m_shooter, constants::shooter::speakerShootSpeed));

  // pathplanner::PPHolonomicDriveController::setRotationTargetOverride([this] () {
  //   return m_vision.GetTargetAngle();
  // });
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
        units::radians_per_second_t angularVelocity = -(rightX * rightX * rightX) * constants::drive::maxAngularVelocity;

        std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

        if(m_driveController.GetLeftTriggerAxis() > constants::controls::axisDeadband)
        {
            // Note target lock

            // std::optional<units::radian_t> angleToTarget = m_vision.GetTargetAngle();
            // if(angleToTarget)
            // {
            //     fmt::print("Angle to object: {}\n", angleToTarget.value());
            //     m_driveBase.TrackObject(angleToTarget.value());
            // }
            // else 
            // {
            //     fmt::print("No target found\n");
            // }

            velocityX = util::sign(leftY) * -(leftY * leftY) * constants::drive::slowMaxDriveVelocity;
            velocityY = util::sign(leftX) * -(leftX * leftX) * constants::drive::slowMaxDriveVelocity;
            angularVelocity = -(rightX * rightX * rightX) * constants::drive::slowMaxAngularVelocity;

        }
        else if(m_driveController.GetRightTriggerAxis() > constants::controls::axisDeadband && alliance.has_value())
        {
            // Speaker/amp target lock

            // frc::Translation2d currentPosition = m_driveBase.GetPose().Translation();

            // frc::Translation2d speakerPosition = GetSpeakerPosition(alliance.value());
            // frc::Translation2d ampPosition = GetSpeakerPosition(alliance.value());

            // frc::Translation2d targetPosition = currentPosition.Nearest({ speakerPosition, ampPosition });
            
            // units::radian_t angleToTarget = units::math::atan2(
            //     targetPosition.Y() - currentPosition.Y(),
            //     targetPosition.X() - currentPosition.X());

            // m_driveBase.TrackObject(angleToTarget);
        }
        else 
        {
            m_driveBase.DisableTracking();
        }

        m_driveBase.Drive(velocityX, velocityY, angularVelocity, !m_driveBase.IsTrackingEnabled());
    },
    {&m_driveBase}
  ));

  frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kX)
    .OnTrue(frc2::cmd::RunOnce([this] { m_driveBase.ZeroHeading(); }, {}));
}

void RobotContainer::ConfigureShooterControls()
{
  // Run intake while A is held
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kA)
    .WhileTrue(IntakeCommands::RunIntake(&m_intake));

  // Shoot to speaker with X
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kX)
     .OnTrue(ShooterCommands::ShootToSpeaker(&m_shooter, &m_intake));
     //.OnTrue(ShooterCommands::Shoot(&m_shooter, &m_intake, 1000_rpm));
    
  // Shoot to amp with Y
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kY)
    .OnTrue(ShooterCommands::ShootToAmp(&m_shooter, &m_intake));
    //.OnTrue(ShooterCommands::RunShooterWheels(&m_shooter, 800_rpm));
    // .OnTrue(ShooterCommands::RunShooterWheels(&m_shooter, 2000_rpm))
    // .OnFalse(ShooterCommands::StopShooterWheels(&m_shooter));

  // Eject from intake with B
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kB)
    .OnTrue(IntakeCommands::EjectIntake(&m_intake, &m_shooter))
    .OnFalse(IntakeCommands::StopIntake(&m_intake))
    .OnFalse(ShooterCommands::StopFeeder(&m_shooter));

  // Intake from shooter with left trigger
  frc2::Trigger([this] { return m_shooterController.GetLeftTriggerAxis() > constants::controls::axisDeadband; })
    .WhileTrue(ShooterCommands::ShooterIntake(&m_shooter, &m_intake));

  // Raise climber with left bumper
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(ClimberCommands::RaiseHook(&m_climber))
    .OnFalse(ClimberCommands::StopClimber(&m_climber));

  // Lower climber with right bumper
  frc2::JoystickButton(&m_shooterController, frc::XboxController::Button::kRightBumper)
    .OnTrue(ClimberCommands::LowerHook(&m_climber))
    .OnFalse(ClimberCommands::StopClimber(&m_climber));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  std::string autoName = m_autoChooser.GetSelected();

  if(autoName == constants::autonomous::noneAuto || autoName == "")
    return frc2::cmd::None();

  if(autoName == constants::autonomous::oneNoteAuto)
    return ShooterCommands::ShootToSpeaker(&m_shooter, &m_intake);

  frc::Pose2d startingPose = pathplanner::PathPlannerAuto::getStartingPoseFromAutoFile(autoName);  
  // startingPose = frc::Pose2d(startingPose.X(), startingPose.Y(), m_driveBase.GetPose().Rotation());
  m_driveBase.ResetPose(startingPose);

  return pathplanner::PathPlannerAuto(autoName).ToPtr()
    .OnlyWhile([&] { return m_driveBase.IsNavXAvailable(); })
    .FinallyDo([&] { m_driveBase.Stop(); });
}

void RobotContainer::InitSysId()
{
  auto routine = m_shooter.GetSysIdRoutine();

  frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kA)
     .OnTrue(m_shooter.GetSysIdRoutine());

  frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kX)
     .WhileTrue(m_driveBase.GetSysIdRoutine());
}