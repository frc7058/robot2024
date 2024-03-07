#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "constants/VisionConstants.h"

class Vision
{
public:
private:
    photon::PhotonCamera m_frontCamera { constants::vision::frontCameraName };
    photon::PhotonCamera m_backCamera { constants::vision::backCameraName };
};