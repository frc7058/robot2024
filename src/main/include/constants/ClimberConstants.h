#pragma once

#include <units/voltage.h>
#include <units/current.h>

namespace constants
{
    namespace climber 
    {
        constexpr units::volt_t motorVoltage = 8.0_V;
        constexpr units::ampere_t currentThreshold = 30.0_A;

        constexpr double minMotorPositionTicks = 0;
        constexpr double maxMotorPositionTicks = 425;
    }
}