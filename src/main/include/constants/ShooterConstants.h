#pragma once

#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

namespace constants 
{
    namespace shooter
    {
        // Maximum shooter voltage
        constexpr units::volt_t maxVoltage = 12.0_V;

        // Shooter velocity control tolerance
        constexpr units::revolutions_per_minute_t tolerance = 10.0_rpm;

        // Maximum time to wait for shooter wheels to reach desired speed
        constexpr units::second_t speedControlTimeout = 1.0_s;

        // Voltage to run the feed motor at 
        constexpr units::volt_t feedMotorPower = 8.0_V;

        // Constant voltage to shoot at (for testing)
        constexpr units::volt_t shootPower = 12.0_V;

        // PID constants 
        namespace pid 
        {
            constexpr double p = 0.0;
            constexpr double i = 0.1;
            constexpr double d = 0.0;
        }

        // Feedforward constants
        namespace feedforward 
        {
            constexpr auto s = 0.1_V; 
            constexpr auto v = 0.5_V * 1.0_s / 1.0_rad;
            constexpr auto a = 0.0_V * 1.0_s * 1.0_s / 1.0_rad;
        }
    }
}