#include "commands/DriveCommand.h"
#include "constants/GeneralConstants.h"
#include "constants/DriveConstants.h"
#include "lib/FieldUtil.h"
#include "lib/Util.h"
#include <frc/MathUtil.h>
#include <units/math.h>

frc2::CommandPtr DriveCommand(DriveBase* driveBase, Vision& vision, frc::XboxController& driveController)
{
    return frc2::cmd::Run(
        [&] {
            double leftX = frc::ApplyDeadband(driveController.GetLeftX(), constants::controls::joystickDeadband);
            double leftY = frc::ApplyDeadband(driveController.GetLeftY(), constants::controls::joystickDeadband);
            double rightX = frc::ApplyDeadband(driveController.GetRightX(), constants::controls::joystickDeadband);

            units::meters_per_second_t velocityX = util::sign(leftY) * -(leftY * leftY) * constants::drive::maxDriveVelocity;
            units::meters_per_second_t velocityY = util::sign(leftX) * -(leftX * leftX) * constants::drive::maxDriveVelocity;
            units::radians_per_second_t angularVelocity = util::sign(rightX) * -(rightX * rightX) * constants::drive::maxAngularVelocity;

            std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

            if(driveController.GetLeftBumper())
            {
                // Note target lock

                std::optional<units::radian_t> angleToTarget = vision.GetTargetAngle();
                if(angleToTarget)
                {
                    fmt::print("Angle to object: {}\n", angleToTarget.value());
                    //m_driveBase.TrackObject(angleToTarget.value());
                }
                else 
                {
                    fmt::print("No target found\n");
                }
            }
            else if(driveController.GetRightBumper() && alliance.has_value())
            {
                // Speaker/amp target lock

                frc::Translation2d currentPosition = driveBase->GetPose().Translation();

                frc::Translation2d speakerPosition = GetSpeakerPosition(alliance.value());
                frc::Translation2d ampPosition = GetSpeakerPosition(alliance.value());

                frc::Translation2d targetPosition = currentPosition.Nearest({ speakerPosition, ampPosition });
                units::radian_t angleToTarget = units::math::atan2(
                    targetPosition.Y() - currentPosition.Y(),
                    targetPosition.X() - currentPosition.X());

                // m_driveBase.TrackObject(angleToTarget);
            }
            else 
            {
                driveBase->DisableTracking();
            }

            driveBase->Drive(velocityX, velocityY, angularVelocity, !driveBase->IsTrackingEnabled());
        },

        {driveBase}
    );
}