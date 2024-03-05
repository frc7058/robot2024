#pragma once

#include <units/voltage.h>
#include <units/time.h>

namespace constants 
{
    namespace intake 
    {
        constexpr units::volt_t intakePower = 12.0_V;
        constexpr units::volt_t feedToShooterPower = 4.0_V;

        constexpr units::second_t feedToShooterTime = 0.5_s;
    }
}