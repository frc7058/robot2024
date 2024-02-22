#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <array>

#include "swerve/SwerveModule.h"
#include "NavX.h"
#include "Constants.h"

class DriveBase : public frc2::SubsystemBase 
{
public:
    DriveBase();

    void Periodic() override;

    void Drive(units::meters_per_second_t velocityX, 
               units::meters_per_second_t velocityY, 
               units::radians_per_second_t angularVelocity, 
               bool fieldRelative);

    void Drive(frc::ChassisSpeeds chassisSpeeds);

    void SetTargetModuleStates(const wpi::array<frc::SwerveModuleState, 4>& moduleStates);

    units::radian_t GetHeading();
    void ZeroHeading();

    frc::Pose2d GetPose() const;
    void ResetPose(frc::Pose2d pose);

    frc::ChassisSpeeds GetChassisSpeeds() const;

    wpi::array<frc::SwerveModuleState, 4> GetSwerveModuleStates() const;
    wpi::array<frc::SwerveModulePosition, 4> GetSwerveModulePositions() const;

    void InitializePreferences();
    void LoadPreferences();

    // Swerve modules (ordered clockwise starting at front left module)
    std::array<std::unique_ptr<SwerveModule>, 4> m_swerveModules {};

private:
    // Swerve kinematics helper class 
    frc::SwerveDriveKinematics<4> m_kinematics {
        frc::Translation2d( constants::drive::moduleDistanceX,  constants::drive::moduleDistanceY),
        frc::Translation2d( constants::drive::moduleDistanceX, -constants::drive::moduleDistanceY),
        frc::Translation2d(-constants::drive::moduleDistanceX,  constants::drive::moduleDistanceY),
        frc::Translation2d(-constants::drive::moduleDistanceX, -constants::drive::moduleDistanceY)};

    // Swerve odometry
    std::unique_ptr<frc::SwerveDriveOdometry<4>> m_odometry {};

    // NavX IMU 
    NavX m_navX;
};