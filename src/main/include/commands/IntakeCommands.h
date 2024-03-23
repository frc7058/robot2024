#pragma once

#include <frc2/command/Commands.h>
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

namespace IntakeCommands 
{
    // Runs the intake until a note is detected
    frc2::CommandPtr RunIntake(Intake* intake);

    // Feeds note from intake into the shooter
    // frc2::CommandPtr FeedShooter(Intake* intake);

    frc2::CommandPtr EjectIntake(Intake* intake, Shooter* shooter);

    frc2::CommandPtr RunFeeder(Intake* intake);
    frc2::CommandPtr StopIntake(Intake* intake);
}