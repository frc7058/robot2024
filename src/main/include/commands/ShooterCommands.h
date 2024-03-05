#pragma once

#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

namespace ShooterCommands 
{
    // Runs the shooter wheels at a specified speed and ends once reached 
    frc2::CommandPtr RunShooterWheels(Shooter* shooter, units::revolutions_per_minute_t speed);

    frc2::CommandPtr RunShooterWheelsConstantVoltage(Shooter* shooter, units::volt_t voltage);

    // Stops the shooter wheels 
    frc2::CommandPtr StopShooterWheels(Shooter* shooter);

    // Runs the feeder wheel 
    frc2::CommandPtr RunFeeder(Shooter* shooter);

    // Stops the feeder wheel
    frc2::CommandPtr StopFeeder(Shooter* shooter);

    // Shoots a note at a specified speed
    frc2::CommandPtr Shoot(Shooter* shooter, Intake* intake, units::revolutions_per_minute_t speed);
    frc2::CommandPtr ShootTest(Shooter* shooter, Intake* intake);
}