#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <memory>
#include <vector>
#include <optional>
#include "constants/VisionConstants.h"

struct VisionPoseResult
{
    photon::EstimatedRobotPose estimatedPose;
    wpi::array<double, 3U> standardDeviations;
};

class Vision
{
public:
    Vision();

    std::optional<VisionPoseResult> GetEstimatedPose(photon::PhotonPoseEstimator& poseEstimator, frc::Pose3d prevPose);
    std::vector<std::optional<VisionPoseResult>> GetEstimatedPoses(frc::Pose3d prevPose);

    std::optional<units::radian_t> GetTargetAngle();

private:
    frc::AprilTagFieldLayout m_fieldLayout { frc::LoadAprilTagLayoutField(constants::vision::aprilTagField) };

    std::shared_ptr<photon::PhotonCamera> m_frontCamera;
    // std::shared_ptr<photon::PhotonCamera> m_backCamera;

    std::unique_ptr<photon::PhotonPoseEstimator> m_frontEstimator;
    // std::unique_ptr<photon::PhotonPoseEstimator> m_backEstimator;

    photon::PhotonCamera m_objectDetectionCamera {constants::vision::objectCamera::name};
};