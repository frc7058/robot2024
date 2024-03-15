#pragma once

#include <frc2/command/Commands.h>
#include "subsystems/Climber.h"

namespace ClimberCommands 
{
    frc2::CommandPtr RaiseHook(Climber* climber);

    frc2::CommandPtr LowerHook(Climber* climber);

    frc2::CommandPtr StopClimber(Climber* climber);
}