#pragma once

#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <units/time.h>
#include <string>
#include "constants/DriveConstants.h"

namespace constants 
{
    namespace autonomous 
    {
        const std::string noneAuto = "None";
        const std::string defaultAuto = noneAuto;
        const std::string oneNoteAuto = "One Note";

        const std::string autoNames[] = {
            "Top - Two Note",
            "Top - Three Note",
            "Middle - Two Note",
            "Middle - Three Note Top",
            "Bottom - Two Note",
            "Bottom - Two Note Far",
            "Bottom - Three Note"
        };

        constexpr pathplanner::HolonomicPathFollowerConfig pathFollowerConfig(
            // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0),

            // Rotation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0),

            // Max swerve module velocity
            constants::drive::maxDriveVelocity,

            // Distance from robot center to swerve modules
            constants::drive::radiusToModules,

            // Default path replanning config
            pathplanner::ReplanningConfig()
        );

        constexpr units::second_t intakeTimeLimit = 3.0_s;
    }
}