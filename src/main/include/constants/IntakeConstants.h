#pragma once

#include <units/voltage.h>
#include <units/time.h>

namespace constants 
{
    namespace intake 
    {
        constexpr units::volt_t intakePower = 0.0_V;
        constexpr units::volt_t feedToShooterPower = 0.0_V;

        constexpr units::second_t feedToShooterTime = 0.5_s;
    }
}