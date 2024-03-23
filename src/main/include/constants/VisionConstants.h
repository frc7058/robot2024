#pragma once

#include <photon/PhotonPoseEstimator.h>
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
        // Main pose strategy
        constexpr photon::PoseStrategy poseStrategy = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;

        // Backup pose strategy (when only one tag is detected)
        constexpr photon::PoseStrategy backupPoseStrategy = photon::PoseStrategy::LOWEST_AMBIGUITY;

        // Max pose ambiguity (for single tag)
        constexpr double maxAmbiguity = 0.2;
        constexpr double maxMultiTagAmbiguity = 0.3;

        // Max distance (for single tag)
        constexpr units::meter_t maxTagDistance = 4.0_m;
  
        // Default standard deviations 
        constexpr wpi::array<double, 3U> singleTagStdDevs {4.0, 4.0, 8.0};
        constexpr wpi::array<double, 3U> multiTagStdDevs {0.8, 0.8, 1.6};

        // Standard deviations scale factor (scaled as: distance^2 * stdDevsScaleFactor)
        constexpr double stdDevsScaleFactor = 1.0 / 30.0;

        namespace frontCamera 
        {
            constexpr std::string_view name = "Front Arducam";
            const frc::Translation3d relativePosition {2.0_in, -6.25_in, 16.0_in};
            const frc::Rotation3d relativeRotation {0_deg, -27.0_deg, 0.0_deg};
        }

        namespace backCamera
        {
            constexpr std::string_view name = "Back Arducam";
            const frc::Translation3d relativePosition {-12.75_in, 7.0_in, 9.5_in};
            const frc::Rotation3d relativeRotation {0_deg, -28.0_deg, 180.0_deg};

            // 28 deg back
        }

        namespace objectCamera 
        {
            constexpr std::string_view name = "Object Detection Camera";
        }
    }
}