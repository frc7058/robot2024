#pragma once

#include <units/voltage.h>
#include <units/current.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>

namespace constants 
{
    namespace shooter
    {
        // Maximum shooter voltage
        constexpr units::volt_t maxVoltage = 12.0_V;

        // Shooter velocity control tolerance
        constexpr units::turns_per_second_t tolerance = 30_rpm;
        constexpr units::millisecond_t debounceTime = 40_ms;

        constexpr units::revolutions_per_minute_t speakerShootSpeed = 4000.0_rpm;
        constexpr units::revolutions_per_minute_t ampShootSpeed = 1000.0_rpm;
        
        constexpr units::volt_t speakerShootPower = 7.0_V;
        constexpr units::volt_t ampShootPower = 2.5_V;

        constexpr units::volt_t maxPIDOutput = 0.25_V;

        // Maximum time to wait for shooter wheels to reach desired speed
        constexpr units::second_t speedControlTimeout = 0.75_s;

        // Voltage to run the feed motor at 
        constexpr units::volt_t feedMotorPower = 8.5_V;

        constexpr units::volt_t intakePower = 7.5_V;

        constexpr uint32_t encoderDepth = 8;
        constexpr uint32_t encoderPeriod = 32;

        // PID constants 
        namespace pid 
        {
            constexpr double p = 0.2;
            constexpr double i = 0.00;
            constexpr double d = 0.0;
            constexpr auto v = units::turn_t {400.0} / 1.0_s / 1.0_s;
            constexpr auto a = units::turn_t {800.0} / 1.0_s / 1.0_s / 1.0_s;
            constexpr units::volt_t maxOutput = 0.3_V;            
        }

        // Feedforward constants
        namespace feedforward 
        {
            constexpr auto s = 0.122_V; 
            constexpr auto v = 0.117_V * 1.0_s / units::turn_t {1.0};
            constexpr auto a = 0.0_V * 1.0_s * 1.0_s / units::turn_t {1.0};
        }
    }
}