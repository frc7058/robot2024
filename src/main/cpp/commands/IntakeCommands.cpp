#include "commands/IntakeCommands.h"
#include "constants/IntakeConstants.h"
#include "constants/ShooterConstants.h"
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
            [intake] { return intake->IsNoteDetected(); },

            // Requirements
            {intake}
        ).ToPtr();
    }

    // frc2::CommandPtr FeedShooter(Intake* intake)
    // {
    //     return frc2::cmd::StartEnd(
    //         // Start
    //         [intake] { intake->RunIntake(constants::intake::feedToShooterPower); },

    //         // End
    //         [intake] { intake->StopIntake(); },

    //         // Requirements
    //         {intake}
    //     ).WithTimeout(constants::intake::feedToShooterTime);
    // }

    frc2::CommandPtr EjectIntake(Intake* intake, Shooter* shooter)
    {
        return intake->RunOnce([intake, shooter] { 
            intake->RunIntake(constants::intake::ejectPower); 
            shooter->RunFeeder(-constants::shooter::feedMotorPower);
        });
    }

    frc2::CommandPtr RunFeeder(Intake* intake)
    {
        return intake->RunOnce([intake] { intake->RunIntake(constants::intake::feedToShooterPower); });
    }

    frc2::CommandPtr StopIntake(Intake* intake)
    {
        return intake->RunOnce([intake] { intake->StopIntake(); });
    }
}