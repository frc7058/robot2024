#pragma once

#include <frc2/command/Commands.h>
#include <frc/XboxController.h>
#include "subsystems/DriveBase.h"
#include "lib/vision.h"

frc2::CommandPtr DriveCommand(DriveBase* driveBase, Vision& vision, frc::XboxController& driveController);