#pragma once

#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include "constants/DriveConstants.h"

namespace constants 
{
    namespace autonomous 
    {
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
    }
}