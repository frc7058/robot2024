#pragma once

#include <units/voltage.h>
#include <units/current.h>
#include <units/time.h>

namespace constants 
{
    namespace intake 
    {
        constexpr units::volt_t intakePower = 8.5_V;
        constexpr units::volt_t ejectPower = -10.0_V;
        constexpr units::volt_t feedToShooterPower = 6.0_V;

        constexpr units::second_t feedToShooterTime = 0.5_s;

        constexpr units::millisecond_t noteDetectionDelay = 40_ms;
    }
}