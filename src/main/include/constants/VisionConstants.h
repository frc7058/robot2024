#pragma once

#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/length.h>
#include <units/angle.h>
#include <string>

namespace constants
{
    namespace vision
    {
        constexpr frc::AprilTagField aprilTagField = frc::AprilTagField::k2024Crescendo;
        constexpr photon::PoseStrategy poseStrategy = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;
        constexpr photon::PoseStrategy backupPoseStrategy = photon::PoseStrategy::LOWEST_AMBIGUITY;
        constexpr double maxAmbiguity = 0.2;

        namespace frontCamera 
        {
            constexpr std::string_view name = "Front Arducam";
            const frc::Translation3d relativePosition {0.0_m, 0.0_m, 0.0_m};
            const frc::Rotation3d relativeRotation {0.0_rad, 0.0_rad, 0.0_rad};
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