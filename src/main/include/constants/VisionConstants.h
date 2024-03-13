#pragma once

#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Rotation3d.h>
#include <wpi/array.h>
#include <units/length.h>
#include <units/angle.h>
#include <string>

namespace constants
{
    namespace vision
    {
        // AprilTag field layout
        constexpr frc::AprilTagField aprilTagField = frc::AprilTagField::k2024Crescendo;

        // Main pose strategy
        constexpr photon::PoseStrategy poseStrategy = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;

        // Backup pose strategy (when only one tag is detected)
        constexpr photon::PoseStrategy backupPoseStrategy = photon::PoseStrategy::LOWEST_AMBIGUITY;

        // Max pose ambiguity (for single tag)
        constexpr double maxAmbiguity = 0.2;

        // Max distance (for single tag)
        constexpr units::meter_t maxSingleTagDistance = 4.0_m;
  
        // Default standard deviations 
        constexpr wpi::array<double, 3U> singleTagStdDevs {4.0, 4.0, 8.0};
        constexpr wpi::array<double, 3U> multiTagStdDevs {0.5, 0.5, 1.0};

        // Standard deviations scale factor (scaled as: distance^2 * stdDevsScaleFactor)
        constexpr double stdDevsScaleFactor = 1.0 / 30.0;

        namespace frontCamera 
        {
            // center, .5 inches extended out (so 14 inches)
            // 30 degrees rotated back
            constexpr std::string_view name = "Front Arducam";
            const frc::Translation3d relativePosition {14.0_in, 0.0_m, 2.0_in};
            const frc::Rotation3d relativeRotation {-45.0_deg, 0.0_rad, 0.0_rad};
        }

        namespace backCamera
        {
            constexpr std::string_view name = "Back Arducam";
            const frc::Translation3d relativePosition {0.0_m, 0.0_m, 0.0_m};
            const frc::Rotation3d relativeRotation {0.0_rad, 0.0_rad, 0.0_rad};
        }

        namespace objectCamera 
        {
            constexpr std::string_view name = "Object Detection Camera";
            const frc::Translation3d relativePosition {0.0_m, 0.0_m, 0.0_m};
            const frc::Rotation3d relativeRotation {0.0_rad, 0.0_rad, 0.0_rad};
        }
    }
}