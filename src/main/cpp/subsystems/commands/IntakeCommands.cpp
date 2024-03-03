#include "commands/IntakeCommands.h"

frc2::CommandPtr RunIntakeCommand(Intake* intake)
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

frc2::CommandPtr FeedShooterCommand(Intake* intake)
{
    return frc2::cmd::StartEnd(
        // Start
        [intake] { intake->RunIntake(constants::intake::feedToShooterPower); },

        // End
        [intake] { intake->StopIntake(); },

        // Requirements
        {intake}
    ).WithTimeout(0.5_s);
}