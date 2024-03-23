#include "commands/ClimberCommands.h"
#include "constants/ClimberConstants.h"
#include <frc2/command/StartEndCommand.h>

namespace ClimberCommands
{
    frc2::CommandPtr RaiseHook(Climber* climber)
    {
        return frc2::cmd::StartEnd(
            // Start
            [climber] {
                if(!climber->ReachedMaxPosition())
                    climber->SetVoltage(constants::climber::motorVoltage); 
            },
            
            // End
            [climber] {climber->Stop(); },

            {climber}
        ).Until([climber] { return climber->ReachedMaxPosition(); /*  || climber->CurrentLimitReached(); */ });
    }

    frc2::CommandPtr LowerHook(Climber* climber)
    {
        return frc2::cmd::StartEnd(
            // Start
            [climber] {
                if(!climber->ReachedMinPosition())
                    climber->SetVoltage(-constants::climber::motorVoltage); 
            },
            
            // End
            [climber] {climber->Stop(); },

            {climber}
        ).Until([climber] { return climber->ReachedMinPosition(); /*  || climber->CurrentLimitReached();  */});
    }

    frc2::CommandPtr StopClimber(Climber* climber)
    {
        return climber->RunOnce([climber] {climber->Stop(); } );
    }
}