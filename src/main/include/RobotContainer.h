// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc/PowerDistribution.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/DriveBase.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Climber.h"
#include "lib/Vision.h"
#include "lib/NavX.h"
#include "constants/Ports.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  void InitSysId();

 private:
  void ConfigureDriveControls();
  void ConfigureShooterControls();

  frc::XboxController m_driveController {0};
  frc::XboxController m_shooterController {1};

  frc::PowerDistribution m_pdh {ports::pdh::pdhCAN, frc::PowerDistribution::ModuleType::kRev};

  Vision m_vision {};

  DriveBase m_driveBase {m_vision};
  Intake m_intake {};
  Shooter m_shooter {};
  Climber m_climber {m_pdh};

  frc::SendableChooser<std::string> m_autoChooser;
};
