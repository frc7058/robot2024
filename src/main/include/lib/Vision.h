#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <memory>
#include <vector>
#include <optional>
#include "constants/VisionConstants.h"

class Vision
{
public:
    Vision();

    std::optional<photon::EstimatedRobotPose> GetEstimatedPose(photon::PhotonPoseEstimator& poseEstimator, frc::Pose3d prevPose);
    std::vector<std::optional<photon::EstimatedRobotPose>> GetEstimatedPoses(frc::Pose3d prevPose);

    std::optional<units::radian_t> GetTargetAngle();

private:
    std::shared_ptr<photon::PhotonCamera> m_frontCamera;
    std::shared_ptr<photon::PhotonCamera> m_backCamera;

    std::unique_ptr<photon::PhotonPoseEstimator> m_frontEstimator;
    std::unique_ptr<photon::PhotonPoseEstimator> m_backEstimator;

    photon::PhotonCamera m_objectDetectionCamera {constants::vision::objectCamera::name};
};