#include "lib/Vision.h"

#include <frc/apriltag/AprilTagFields.h>

Vision::Vision()
{
    frc::AprilTagFieldLayout fieldLayout = frc::LoadAprilTagLayoutField(constants::vision::aprilTagField);

    m_frontEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        fieldLayout, 
        constants::vision::poseStrategy, 
        photon::PhotonCamera(constants::vision::frontCamera::name),
        frc::Transform3d(
            constants::vision::frontCamera::relativePosition,
            constants::vision::frontCamera::relativeRotation
        ));

    m_backEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        fieldLayout, 
        constants::vision::poseStrategy, 
        photon::PhotonCamera(constants::vision::backCamera::name),
        frc::Transform3d(
            constants::vision::backCamera::relativePosition,
            constants::vision::backCamera::relativeRotation
        ));

    m_frontCamera = m_frontEstimator->GetCamera();
    m_backCamera = m_frontEstimator->GetCamera();

    m_frontEstimator->SetMultiTagFallbackStrategy(constants::vision::backupPoseStrategy);
    m_backEstimator->SetMultiTagFallbackStrategy(constants::vision::backupPoseStrategy);
}

std::optional<photon::EstimatedRobotPose> Vision::GetEstimatedPose(photon::PhotonPoseEstimator& poseEstimator, frc::Pose3d prevPose)
{
    poseEstimator.SetReferencePose(prevPose);
    std::optional<photon::EstimatedRobotPose> estimatedPose = poseEstimator.Update();

    if(estimatedPose.has_value())
    {
        // Check if within field here

        size_t numTargets = estimatedPose->targetsUsed.size();

        // Use pose estimation if more than one target was used
        if(numTargets > 1)
            return estimatedPose;

        // Use pose estimation if one target was used and is below ambiguity threshold
        if(numTargets == 1 && estimatedPose->targetsUsed[0].GetPoseAmbiguity() <= constants::vision::maxAmbiguity)
            return estimatedPose;
    }

    return std::nullopt;
}

std::vector<std::optional<photon::EstimatedRobotPose>> Vision::GetEstimatedPoses(frc::Pose3d prevPose)
{
    std::vector<std::optional<photon::EstimatedRobotPose>> poses;

    poses.push_back(GetEstimatedPose(*m_frontEstimator, prevPose));
    poses.push_back(GetEstimatedPose(*m_backEstimator, prevPose));

    return poses;
}