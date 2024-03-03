#pragma once

#include <units/angle.h>

namespace constants 
{
    constexpr uint64_t navXTimeoutSeconds = 30.0;

    constexpr double pi = 3.14159265358979323846;
    constexpr units::radian_t pi_radians { pi };

    namespace controls 
    {
        constexpr double joystickDeadband = 0.08;
    }
}