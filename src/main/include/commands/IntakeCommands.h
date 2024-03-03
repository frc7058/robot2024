#pragma once

#include <frc2/command/Commands.h>
#include "subsystems/Intake.h"

namespace IntakeCommands 
{
    // Runs the intake until a note is detected
    frc2::CommandPtr RunIntake(Intake* intake);

    // Feeds note from intake into the shooter
    frc2::CommandPtr FeedShooter(Intake* intake);

    // Eject note from intake
    // frc2::CommandPtr Eject(Intake* intake);
}