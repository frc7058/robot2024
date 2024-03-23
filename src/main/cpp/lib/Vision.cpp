#include "lib/Vision.h"

#include <frc/apriltag/AprilTagFields.h>
#include <wpinet/PortForwarder.h>
#include "constants/GeneralConstants.h"

Vision::Vision()
{
    m_frontEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        m_fieldLayout, 
        constants::vision::poseStrategy, 
        photon::PhotonCamera(constants::vision::frontCamera::name),
        frc::Transform3d(
            constants::vision::frontCamera::relativePosition,
            constants::vision::frontCamera::relativeRotation
        ));

    m_backEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        m_fieldLayout, 
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

    wpi::PortForwarder::GetInstance().Add(5800, "photonvision.local", 5800);
}

std::optional<VisionPoseResult> Vision::GetEstimatedPose(photon::PhotonPoseEstimator& poseEstimator, frc::Pose3d prevPose)
{
    poseEstimator.SetReferencePose(prevPose);
    std::optional<photon::EstimatedRobotPose> estimatedPose = poseEstimator.Update();

    if(estimatedPose.has_value())
    {
        units::meter_t x = estimatedPose->estimatedPose.X();
        units::meter_t y = estimatedPose->estimatedPose.Y();

        // Discard estimation if outside field bounds
        if(x < 0_m && x > constants::field::lengthX &&
           y < 0_m && y > constants::field::lengthY)
        {
            return std::nullopt;
        }

        size_t numTargets = estimatedPose->targetsUsed.size();
        frc::Pose2d estimatedPose2D = estimatedPose->estimatedPose.ToPose2d();

        units::meter_t averageDistance = 0.0_m;
        double averageAmbiguity = 0.0;

        for(const photon::PhotonTrackedTarget& target : estimatedPose->targetsUsed)
        {
            auto tagPose = m_fieldLayout.GetTagPose(target.GetFiducialId());

            if(tagPose.has_value())
            {
                averageDistance += tagPose.value().ToPose2d().Translation().Distance(estimatedPose2D.Translation());
                averageAmbiguity += target.GetPoseAmbiguity();
            }
        }

        averageDistance /= numTargets;
        averageAmbiguity /= numTargets;
        
        double doubleMax = std::numeric_limits<double>::max();
        wpi::array<double, 3U> stdDevs { doubleMax, doubleMax, doubleMax };

        if(numTargets > 1 && averageAmbiguity <= constants::vision::maxMultiTagAmbiguity && averageDistance <= constants::vision::maxTagDistance)
        {
            // Use pose estimation if more than one target was used
            stdDevs = constants::vision::multiTagStdDevs;

            fmt::print("Using two targets\n");
        }
        else if(numTargets == 1 && 
            averageAmbiguity <= constants::vision::maxAmbiguity &&
            averageDistance <= constants::vision::maxTagDistance)
        {
            // Use pose estimation if one target was used, is below ambiguity threshold, and is within max range
            stdDevs = constants::vision::singleTagStdDevs;
            
            fmt::print("Using single target\n");
        }
        else 
        {
            // Discard pose estimation otherwise
            return std::nullopt;

            fmt::print("Discarding target\n");
        }

        // Scale standard deviations by average tag distance squared
        double scaleFactor = 1.0 + (averageDistance.value() * averageDistance.value() * constants::vision::stdDevsScaleFactor);
        for(double& stdDev : stdDevs)
        {
            stdDev *= scaleFactor;
        }

        return VisionPoseResult {estimatedPose.value(), stdDevs};
    }

    return std::nullopt;
}

std::vector<std::optional<VisionPoseResult>> Vision::GetEstimatedPoses(frc::Pose3d prevPose)
{
    std::vector<std::optional<VisionPoseResult>> poses;

    poses.push_back(GetEstimatedPose(*m_frontEstimator, prevPose));
    poses.push_back(GetEstimatedPose(*m_backEstimator, prevPose));

    return poses;
}

std::optional<units::radian_t> Vision::GetTargetAngle()
{
    photon::PhotonPipelineResult result = m_objectDetectionCamera.GetLatestResult();

    if(result.HasTargets())
    {
        units::degree_t angle {result.GetBestTarget().GetYaw()};
        return units::radian_t {angle};
    }

    return std::nullopt;
}