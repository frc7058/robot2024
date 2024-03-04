#include "commands/IntakeCommands.h"
#include "constants/IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>

namespace IntakeCommands 
{
    frc2::CommandPtr RunIntake(Intake* intake)
    {
        return frc2::FunctionalCommand(
            // Initialize
            [intake] { intake->RunIntake(constants::intake::intakePower); },

            // Periodic
            [intake] {},

            // OnEnd
            [intake] (bool interrupted) { intake->StopIntake(); },

            // IsFinished
            [intake] { return false; /* return intake->IsNoteDetected(); */ },

            // Requirements
            {intake}
        ).ToPtr();
    }

    frc2::CommandPtr FeedShooter(Intake* intake)
    {
        return frc2::cmd::StartEnd(
            // Start
            [intake] { intake->RunIntake(constants::intake::feedToShooterPower); },

            // End
            [intake] { intake->StopIntake(); },

            // Requirements
            {intake}
        ).WithTimeout(constants::intake::feedToShooterTime);
    }

    frc2::CommandPtr Eject(Intake* intake)
    {
        return frc2::cmd::StartEnd(
            // Start 
            [intake] { intake->RunIntake(constants::intake::ejectPower); },

            // End
            [intake] { intake->StopIntake(); },

            // Requirements
            {intake}
        );
    }
}