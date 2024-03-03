#pragma once

#include <units/length.h>
#include <numbers>

namespace constants 
{
    namespace physical 
    {
        // Swerve wheels inforrmation
        constexpr units::meter_t wheelDiameter = 0.1016_m;
        constexpr units::meter_t wheelCircumference = wheelDiameter * std::numbers::pi;

        // Swerve module locations from center of mass 
        constexpr units::meter_t moduleDistanceX = 0.27305_m; 
        constexpr units::meter_t moduleDistanceY = 0.27305_m; 
        constexpr units::meter_t radiusToModules = 0.38615_m; 

        // Swerve drive gear ratio 
        constexpr double driveGearRatio = 6.75;
    }
}