#include "commands/ShooterCommands.h"
#include "commands/IntakeCommands.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitCommand.h>
#include "constants/ShooterConstants.h"

namespace ShooterCommands 
{
    frc2::CommandPtr RunShooterWheels(Shooter* shooter, units::revolutions_per_minute_t speed)
    {
        return frc2::cmd::Run(
            [shooter, speed] { shooter->SetShooterSpeed(speed); },
            {shooter}
        ).Until([shooter] { return shooter->AtSpeed(); });
    }

    frc2::CommandPtr RunShooterWheelsConstantVoltage(Shooter* shooter, units::volt_t voltage)
    {
        return shooter->RunOnce([shooter, voltage] { shooter->SetShooterVoltage(voltage); });
    }

    frc2::CommandPtr StopShooterWheels(Shooter* shooter)
    {
        return shooter->RunOnce([shooter] { shooter->StopShooter(); });
    }

    frc2::CommandPtr RunFeeder(Shooter* shooter)
    {
        return shooter->RunOnce([shooter] { shooter->RunFeeder(constants::shooter::feedMotorPower); });
    }

    frc2::CommandPtr StopFeeder(Shooter* shooter)
    {
        return shooter->RunOnce([shooter] { shooter->StopFeeder(); });
    }

    frc2::CommandPtr Shoot(Shooter* shooter, Intake* intake, units::revolutions_per_minute_t speed)
    {
        /*
        return ShooterCommands::RunShooterWheels(shooter, speed).WithTimeout(constants::shooter::speedControlTimeout)
            .AlongWith(ShooterCommands::RunFeeder(shooter))
            .AndThen(IntakeCommands::FeedShooter(intake))
            .AndThen(ShooterCommands::StopShooterWheels(shooter))
            .AndThen(ShooterCommands::StopFeeder(shooter));
        */
       return frc2::cmd::None();
    }

    frc2::CommandPtr ShootTest(Shooter* shooter, Intake* intake)
    {
        return ShooterCommands::RunShooterWheelsConstantVoltage(shooter, constants::shooter::shootPower)
            .AndThen(frc2::cmd::Wait(0.5_s))
            .AndThen(
                IntakeCommands::RunFeeder(intake)
                    .AlongWith(ShooterCommands::RunFeeder(shooter))
            )
            .AndThen(frc2::cmd::Wait(0.4_s))
            .AndThen(IntakeCommands::StopIntake(intake))
            .AndThen(ShooterCommands::StopFeeder(shooter))
            .AndThen(ShooterCommands::StopShooterWheels(shooter));
    }
}