#pragma once

#include <units/voltage.h>
#include <units/current.h>
#include <units/time.h>

namespace constants 
{
    namespace intake 
    {
        constexpr units::ampere_t maxCurrent = 30.0_A;

        constexpr units::volt_t intakePower = 8.0_V;
        constexpr units::volt_t feedToShooterPower = 4.0_V;

        constexpr units::second_t feedToShooterTime = 0.5_s;
    }
}