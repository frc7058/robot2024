#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>

namespace constants 
{
    namespace field 
    {
        // AprilTag field layout
        constexpr frc::AprilTagField aprilTagField = frc::AprilTagField::k2024Crescendo;

        // Field dimensions from page 21 of the manual (https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf)
        constexpr units::meter_t lengthX = 16.54175_m;
        constexpr units::meter_t lengthY = 8.21055_m;

        constexpr frc::Translation2d redSpeakerPosition { 652.73_in, 218.42_in };
        constexpr frc::Translation2d redAmpPosition { 578.77_in, 323.00_in };

        constexpr frc::Translation2d blueSpeakerPosition { -1.50_in, 218.42_in };
        constexpr frc::Translation2d blueAmpPosition { 72.50_in, 323.00_in };
    }
}