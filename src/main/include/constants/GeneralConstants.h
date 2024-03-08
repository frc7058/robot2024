#pragma once

#include <units/angle.h>
#include <units/length.h>

namespace constants 
{
    constexpr uint64_t navXTimeoutSeconds = 30.0;

    constexpr double pi = 3.14159265358979323846;
    constexpr units::radian_t piRadians { pi };

    namespace controls 
    {
        constexpr double joystickDeadband = 0.08;
    }

    namespace field 
    {
        constexpr units::meter_t lengthX = 100.0_m;
        constexpr units::meter_t lengthY = 100.0_m;
    }
}