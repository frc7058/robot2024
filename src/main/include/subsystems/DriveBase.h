#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
// #include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/PIDController.h>
#include <AHRS.h>
#include <array>

#include "lib/SwerveModule.h"
#include "lib/NavX.h"
#include "lib/Vision.h"

class DriveBase : public frc2::SubsystemBase 
{
public:
    DriveBase(Vision& vision);

    void Periodic() override;

    void Drive(units::meters_per_second_t velocityX, 
               units::meters_per_second_t velocityY, 
               units::radians_per_second_t angularVelocity, 
               bool fieldRelative);

    void Drive(frc::ChassisSpeeds chassisSpeeds);

    void Stop();

    void SetTargetModuleStates(const wpi::array<frc::SwerveModuleState, 4>& moduleStates);

    void VisionUpdate();

    void TrackObject(units::radian_t heading);
    void DisableTracking();

    // void LockHeading();
    // void UnlockHeading();
    // bool IsHeadingLocked() const;

    units::radian_t GetHeading();
    void ZeroHeading();

    frc::Pose2d GetPose() const;
    void ResetPose(frc::Pose2d pose);

    frc::ChassisSpeeds GetChassisSpeeds() const;

    wpi::array<frc::SwerveModuleState, 4> GetSwerveModuleStates() const;
    wpi::array<frc::SwerveModulePosition, 4> GetSwerveModulePositions() const;

    bool IsNavXAvailable();

    void InitializePreferences();
    void LoadPreferences();

    // frc2::sysid::SysIdRoutine GetSysIdRoutine();

private:
    // Swerve modules (ordered clockwise starting at front left module)
    std::array<std::unique_ptr<SwerveModule>, 4> m_swerveModules {};

    // Swerve kinematics helper class 
    std::unique_ptr<frc::SwerveDriveKinematics<4>> m_kinematics {};

    // Swerve odometry
    std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> m_poseEstimator {};
    // std::unique_ptr<frc::SwerveDriveOdometry<4>> m_odometry {};

    // NavX IMU 
    // NavX& m_navX;
    AHRS m_navX {frc::SerialPort::kUSB1};

    // Vision class for AprilTag pose estimation
    Vision& m_vision;

    // PID controller to lock/maintain heading
    std::unique_ptr<frc::PIDController> m_headingPID {};
    bool m_tracking = false;

    frc::Field2d m_field;
};