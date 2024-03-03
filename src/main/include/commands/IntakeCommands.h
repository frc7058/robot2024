#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "subsystems/Intake.h"

// Runs the intake until a note is detected
frc2::CommandPtr RunIntakeCommand(Intake* intake);

// Feeds note from intake into the shooter
frc2::CommandPtr FeedShooterCommand(Intake* intake);