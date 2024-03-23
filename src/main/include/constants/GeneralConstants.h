#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <frc/util/Color8Bit.h>

namespace constants 
{
    constexpr uint64_t navXTimeoutSeconds = 30.0;

    constexpr double pi = 3.14159265358979323846;
    constexpr units::radian_t piRadians { pi };

    constexpr bool enableSysId = false;

    namespace controls 
    {
        constexpr double joystickDeadband = 0.08;
        constexpr double axisDeadband = 0.05;
    }
    
    namespace LED
    {
        constexpr int length = 1;

        namespace colors
        {
            constexpr frc::Color8Bit blue(0, 47, 135);
            constexpr frc::Color8Bit gold(255, 215, 0);

            constexpr frc::Color8Bit red(255, 0, 0);
            constexpr frc::Color8Bit green(0, 255, 0);
        }
    }
}