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
        // Field dimensions from page 21 of the manual (https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf)
        constexpr units::meter_t lengthX = 16.54175_m;
        constexpr units::meter_t lengthY = 8.21055_m;
    }

    constexpr bool enableSysID = true;
}