#include "lib/FieldUtil.h"
#include "constants/FieldConstants.h"

frc::Translation2d GetSpeakerPosition(frc::DriverStation::Alliance alliance)
{
    return alliance == frc::DriverStation::Alliance::kRed
        ? constants::field::redSpeakerPosition
        : constants::field::blueSpeakerPosition;
}

frc::Translation2d GetAmpPosition(frc::DriverStation::Alliance alliance)
{
    return alliance == frc::DriverStation::Alliance::kRed
        ? constants::field::redAmpPosition
        : constants::field::blueAmpPosition;
}