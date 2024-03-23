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
        return shooter->RunOnce([shooter, speed ] { shooter->SetShooterSpeed(speed); })
            .AndThen(frc2::cmd::WaitUntil([shooter] { return shooter->AtSpeed(); }));
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
        return ShooterCommands::RunShooterWheels(shooter, speed).WithTimeout(constants::shooter::speedControlTimeout)
            .AndThen(
                IntakeCommands::RunFeeder(intake)
                    .AlongWith(ShooterCommands::RunFeeder(shooter))
            )
            .AndThen(
                frc2::cmd::WaitUntil([intake] { return !intake->IsNoteDetected(); })
                    .WithTimeout(0.5_s)
            )
            .AndThen(frc2::cmd::Wait(0.75_s))
            .FinallyDo([intake, shooter] {
                intake->StopIntake();
                shooter->StopShooter();
                shooter->StopFeeder();
            });
    }

    frc2::CommandPtr Shoot(Shooter* shooter, Intake* intake, units::volt_t voltage)
    {
        return ShooterCommands::RunShooterWheelsConstantVoltage(shooter, voltage)
            .AndThen(frc2::cmd::Wait(0.5_s))
            .AndThen(IntakeCommands::RunFeeder(intake))
            .AndThen(ShooterCommands::RunFeeder(shooter))
            .AndThen(frc2::cmd::Wait(0.5_s))
            .AndThen(IntakeCommands::StopIntake(intake))
            .AndThen(ShooterCommands::StopShooterWheels(shooter))
            .AndThen(ShooterCommands::StopFeeder(shooter));
    }

    frc2::CommandPtr ShootToSpeaker(Shooter* shooter, Intake* intake)
    {
        // return ShooterCommands::RunShooterWheels(shooter, 1000_rpm);
        //return ShooterCommands::Shoot(shooter, intake, constants::shooter::speakerShootPower);
        return ShooterCommands::RunShooterWheelsConstantVoltage(shooter, constants::shooter::speakerShootPower)
            .AndThen(frc2::cmd::Wait(0.75_s))
            .AndThen(
                IntakeCommands::RunFeeder(intake)
                    .AlongWith(ShooterCommands::RunFeeder(shooter))
            )
            .AndThen(frc2::cmd::Wait(0.7_s))
            .FinallyDo([intake, shooter] {
                intake->StopIntake();
                shooter->StopShooter();
                shooter->StopFeeder();
            });
    }

    frc2::CommandPtr ShootToAmp(Shooter* shooter, Intake* intake)
    {
        return ShooterCommands::RunShooterWheelsConstantVoltage(shooter, constants::shooter::ampShootPower)
            .AndThen(frc2::cmd::Wait(0.65_s))
            .AndThen(
                IntakeCommands::RunFeeder(intake)
                    .AlongWith(ShooterCommands::RunFeeder(shooter))
            )
            .AndThen(frc2::cmd::Wait(0.75_s))
            .FinallyDo([intake, shooter] {
                intake->StopIntake();
                shooter->StopShooter();
                shooter->StopFeeder();
            });
        //return ShooterCommands::Shoot(shooter, intake, constants::shooter::ampShootPower);
        // return ShooterCommands::Shoot(shooter, intake, constants::shooter::ampShootSpeed);
    }

    frc2::CommandPtr ShooterIntake(Shooter* shooter, Intake* intake)
    {
        return frc2::cmd::StartEnd(
            // Start
            [shooter, intake] { 
                shooter->SetShooterVoltage(-constants::shooter::intakePower);
                shooter->RunFeeder(-constants::shooter::intakePower);
                intake->RunIntake(4.0_V);
            },

            // End
            [shooter, intake] {
                shooter->StopShooter();
                shooter->StopFeeder();
                intake->StopIntake();
            }
        ); 
        // .Until([intake] { return intake->IsNoteDetected(); });
    }
}