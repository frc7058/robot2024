#include "subsystems/DriveBase.h"

#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "constants/Ports.h"
#include "constants/DriveConstants.h"
#include "constants/PhysicalConstants.h"
#include "constants/AutoConstants.h"
#include "constants/GeneralConstants.h"

DriveBase::DriveBase(Vision& vision)
    : m_vision(vision)
{
    frc::SmartDashboard::PutBoolean("Cosine scaling", true);
    frc::SmartDashboard::PutNumber("Cosine scaling exponent", constants::drive::cosineScalingExponent);

    fmt::print("\nInitializing DriveBase...\n");

    m_swerveModules[0] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (FL)", 
        ports::drive::driveMotorCAN::frontLeft,
        ports::drive::turnMotorCAN::frontLeft,
        ports::drive::CANCoder::frontLeft,
        constants::drive::encoderOffsets::frontLeft));

    m_swerveModules[1] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (FR)", 
        ports::drive::driveMotorCAN::frontRight, 
        ports::drive::turnMotorCAN::frontRight, 
        ports::drive::CANCoder::frontRight,
        constants::drive::encoderOffsets::frontRight));
    m_swerveModules[1]->SetDriveMotorInverted(true);

    m_swerveModules[2] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (BL)", 
        ports::drive::driveMotorCAN::backLeft, 
        ports::drive::turnMotorCAN::backLeft, 
        ports::drive::CANCoder::backLeft,
        constants::drive::encoderOffsets::backLeft));

    m_swerveModules[3] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (BR)", 
        ports::drive::driveMotorCAN::backRight,
        ports::drive::turnMotorCAN::backRight,
        ports::drive::CANCoder::backRight,
        constants::drive::encoderOffsets::backRight));
    m_swerveModules[3]->SetDriveMotorInverted(true);

    // if(m_navX.IsAvailable())
    // {
    //     ZeroHeading();
    // }
    // else 
    // {
    //     fmt::print("NavX is not available: Field-oriented drive and odometry are disabled.\n");
    // }
     
    m_kinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(
        frc::Translation2d( constants::physical::moduleDistanceX,  constants::physical::moduleDistanceY),
        frc::Translation2d( constants::physical::moduleDistanceX, -constants::physical::moduleDistanceY),
        frc::Translation2d(-constants::physical::moduleDistanceX,  constants::physical::moduleDistanceY),
        frc::Translation2d(-constants::physical::moduleDistanceX, -constants::physical::moduleDistanceY));

    m_poseEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(
        *m_kinematics, 
        GetHeading(), 
        GetSwerveModulePositions(), 
        frc::Pose2d(
            frc::Translation2d(0.0_m, 0.0_m),
            frc::Rotation2d(0.0_rad)
        ));

    units::radian_t headingTolerance = 0.25_deg;
    m_headingPID = std::make_unique<frc::PIDController>(3.0, 0, 0);
    m_headingPID->SetTolerance(headingTolerance.value());
    m_headingPID->EnableContinuousInput(-constants::pi, constants::pi);

    InitializePreferences();

    pathplanner::AutoBuilder::configureHolonomic(
        // Get pose
        [this] () { return this->GetPose(); },

        // Reset pose
        [this] (frc::Pose2d pose) { this->ResetPose(pose); },

        // Get robot-relative speeds
        [this] () { return this->GetChassisSpeeds(); },

        // Drive
        [this] (frc::ChassisSpeeds robotRelativeSpeeds) { this->Drive(robotRelativeSpeeds); },

        // Path follower config
        constants::autonomous::pathFollowerConfig,

        // Boolean supplier for path mirroring
        [] () {
            auto alliance = frc::DriverStation::GetAlliance();

            if(alliance)
                return alliance.value() == frc::DriverStation::Alliance::kRed;

            return false;
        },

        this
    );
    
    fmt::print("DriveBase Initialization complete\n\n");
}

void DriveBase::Periodic()
{
    for(std::unique_ptr<SwerveModule>& swerveModule : m_swerveModules)
    {
        swerveModule->Periodic();
    }

    m_poseEstimator->Update(m_navX.GetRotation2d(), GetSwerveModulePositions());

    // VisionUpdate();
}

void DriveBase::Drive(units::meters_per_second_t velocityX, units::meters_per_second_t velocityY, units::radians_per_second_t angularVelocity, bool fieldRelative)
{
    //fmt::print("Vx: {} m/s, Vy: {} m/s, Vangular: {} rad/s\n", velocityX.value(), velocityY.value(), angularVelocity.value());

    frc::ChassisSpeeds chassisSpeeds { velocityX, velocityY, angularVelocity };

    if(fieldRelative)
    {
        frc::Rotation2d robotRotation = m_navX.GetRotation2d();
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, robotRotation);
    }

    Drive(chassisSpeeds);
}

#include <iostream>

void DriveBase::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    // If tracking an object, rotate towards
    // if(m_tracking)
    // {
    //     double heading = GetHeading().value();

    //     units::radians_per_second_t outputAngularVelocity { -m_headingPID->Calculate(heading) };

    //     if(m_headingPID->AtSetpoint())
    //     {
    //         // std::cout << "At setpoint\n";
    //         chassisSpeeds.omega = 0.0_rad_per_s;
    //     }
    //     else 
    //     {
    //         chassisSpeeds.omega = std::min(outputAngularVelocity, constants::drive::maxAngularVelocity);

    //         // units::degree_t degreeError = units::radian_t {m_headingPID->GetPositionError()};         
    //         // fmt::print("Error: {} ({}), output: {}", m_headingPID->GetPositionError(), degreeError, outputAngularVelocity);
    //     }
    // } 

    units::radians_per_second_t angularVelocity = chassisSpeeds.omega;
    chassisSpeeds.omega *= constants::drive::angularVelocityFudgeFactor;
    chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, 0.02_s);
    chassisSpeeds.omega = angularVelocity;

    wpi::array<frc::SwerveModuleState, 4> swerveModuleStates = m_kinematics->ToSwerveModuleStates(chassisSpeeds);
    m_kinematics->DesaturateWheelSpeeds(&swerveModuleStates, constants::drive::maxDriveVelocity);

    for(size_t index = 0; index < m_swerveModules.size(); index++)
    {
        frc::SwerveModuleState& moduleState = swerveModuleStates[index];
        units::radian_t currentTurnAngle = m_swerveModules[index]->GetTurnAngle();

        // Optimize module states to minimize the required turn angle
        moduleState = frc::SwerveModuleState::Optimize(moduleState, frc::Rotation2d(currentTurnAngle));
    
        // Scale module speed by the cosine of angle error (to some power)
        // Reduces motor movement while modules are still turning to their target angle
        if(constants::drive::enableCosineScaling)
        {
            double errorCosine = units::math::cos(currentTurnAngle - moduleState.angle.Radians());
            double scalingFactor = std::pow(std::abs(errorCosine), constants::drive::cosineScalingExponent);

            moduleState.speed *= scalingFactor;
        }
    }

    SetTargetModuleStates(swerveModuleStates);
}

void DriveBase::Stop()
{
    Drive(frc::ChassisSpeeds {0_mps, 0_mps, 0_rad_per_s});
}

void DriveBase::SetTargetModuleStates(const wpi::array<frc::SwerveModuleState, 4>& moduleStates)
{
    // Check if all drive speeds are zero
    /*
    bool moving = false;
    for(const frc::SwerveModuleState& state : moduleStates)
    {
        if(state.speed != 0.0_mps)
        {
            moving = true;
            break;
        }
    }
    */

    for(size_t moduleIndex = 0; moduleIndex < 4; moduleIndex++)
    {
        frc::SwerveModuleState state = moduleStates[moduleIndex];
        m_swerveModules[moduleIndex]->SetTargetState(state);

        /*
        if(moving)
        {
            m_swerveModules[moduleIndex]->SetTargetState(state);
        }
        else 
        {
            // If not moving, set only the drive speed to zero to avoid each module resetting its angle
            m_swerveModules[moduleIndex]->SetDriveVelocity(0.0_mps);
        }
        */
    }
}

void DriveBase::VisionUpdate()
{
    frc::Pose3d currentPose(GetPose());
    std::vector<std::optional<VisionPoseResult>> estimatedPoses = m_vision.GetEstimatedPoses(currentPose);

    for(std::optional<VisionPoseResult>& visionResult : estimatedPoses)
    {
        if(visionResult.has_value())
        {
            frc::Pose2d estimatedPose2d = visionResult->estimatedPose.estimatedPose.ToPose2d();
            units::second_t timestamp = visionResult->estimatedPose.timestamp;
            
            m_poseEstimator->AddVisionMeasurement(estimatedPose2d, timestamp, visionResult->standardDeviations);
        }
    }
}

void DriveBase::TrackObject(units::radian_t heading)
{
    heading = frc::AngleModulus(heading);
    m_headingPID->SetSetpoint(heading.value());
    m_tracking = true;
}

void DriveBase::DisableTracking()
{
    m_tracking = false;
}

units::radian_t DriveBase::GetHeading()
{
    units::degree_t heading {m_navX.GetYaw()};
    return units::radian_t {heading};

    // if(m_navX.IsAvailable())
    // {
    //     units::degree_t heading {m_navX.Get().GetYaw()};
    //     return units::radian_t {heading};
    // }
    // else 
    // {
    //     return units::radian_t {0};
    // }
}

void DriveBase::ZeroHeading()
{
    m_navX.ZeroYaw();
}

frc::Pose2d DriveBase::GetPose() const
{
    return m_poseEstimator->GetEstimatedPosition();
}

void DriveBase::ResetPose(frc::Pose2d pose)
{
    m_poseEstimator->ResetPosition(GetHeading(), GetSwerveModulePositions(), pose);
}

frc::ChassisSpeeds DriveBase::GetChassisSpeeds() const
{
    return m_kinematics->ToChassisSpeeds(GetSwerveModuleStates());
}

wpi::array<frc::SwerveModuleState, 4> DriveBase::GetSwerveModuleStates() const
{
    return {
        m_swerveModules[0]->GetState(),
        m_swerveModules[1]->GetState(),
        m_swerveModules[2]->GetState(),
        m_swerveModules[3]->GetState()
    };
}

wpi::array<frc::SwerveModulePosition, 4> DriveBase::GetSwerveModulePositions() const 
{
    return {
        m_swerveModules[0]->GetPosition(),
        m_swerveModules[1]->GetPosition(),
        m_swerveModules[2]->GetPosition(),
        m_swerveModules[3]->GetPosition()
    };
}

bool DriveBase::IsNavXAvailable()
{
    return m_navX.IsConnected() && !m_navX.IsCalibrating();
}

void DriveBase::InitializePreferences()
{
    frc::Preferences::InitDouble(constants::drive::preferences::driveP_Key, constants::drive::drivePID::p);
    frc::Preferences::InitDouble(constants::drive::preferences::driveI_Key, constants::drive::drivePID::i);
    frc::Preferences::InitDouble(constants::drive::preferences::driveD_Key, constants::drive::drivePID::d);
    frc::Preferences::InitDouble(constants::drive::preferences::driveFF_S_Key, constants::drive::driveFF::s.value());
    frc::Preferences::InitDouble(constants::drive::preferences::driveFF_V_Key, constants::drive::driveFF::v.value());
    
    frc::Preferences::InitDouble(constants::drive::preferences::turnP_Key, constants::drive::turnPID::p);
    frc::Preferences::InitDouble(constants::drive::preferences::turnI_Key, constants::drive::turnPID::i);
    frc::Preferences::InitDouble(constants::drive::preferences::turnD_Key, constants::drive::turnPID::d);
    frc::Preferences::InitDouble(constants::drive::preferences::turnF_Key, constants::drive::turnPID::f);
    frc::Preferences::InitDouble(constants::drive::preferences::turnV_Key, constants::drive::turnPID::maxVelocity.value());
    frc::Preferences::InitDouble(constants::drive::preferences::turnA_Key, constants::drive::turnPID::maxAcceleration.value());

    /*
    frc::Preferences::InitDouble(constants::drive::preferences::offsetFL_Key, constants::drive::encoder_offsets::frontLeft.value());
    frc::Preferences::InitDouble(constants::drive::preferences::offsetFR_Key, constants::drive::encoder_offsets::frontRight.value());
    frc::Preferences::InitDouble(constants::drive::preferences::offsetBL_Key, constants::drive::encoder_offsets::backLeft.value());
    frc::Preferences::InitDouble(constants::drive::preferences::offsetBR_Key, constants::drive::encoder_offsets::backRight.value());
    */

    LoadPreferences();
}

void DriveBase::LoadPreferences()
{
    double driveP = frc::Preferences::GetDouble(constants::drive::preferences::driveP_Key);
    double driveI = frc::Preferences::GetDouble(constants::drive::preferences::driveI_Key);
    double driveD = frc::Preferences::GetDouble(constants::drive::preferences::driveD_Key);
    double driveFF_S = frc::Preferences::GetDouble(constants::drive::preferences::driveFF_S_Key);
    double driveFF_V = frc::Preferences::GetDouble(constants::drive::preferences::driveFF_V_Key);

    double turnP = frc::Preferences::GetDouble(constants::drive::preferences::turnP_Key);
    double turnI = frc::Preferences::GetDouble(constants::drive::preferences::turnI_Key);
    double turnD = frc::Preferences::GetDouble(constants::drive::preferences::turnD_Key);
    double turnF = frc::Preferences::GetDouble(constants::drive::preferences::turnF_Key);
    units::radians_per_second_t turnV { frc::Preferences::GetDouble(constants::drive::preferences::turnV_Key) };
    units::radians_per_second_squared_t turnA { frc::Preferences::GetDouble(constants::drive::preferences::turnA_Key) };

    // units::radian_t offsetFL { frc::Preferences::GetDouble(constants::preferences::offsetFL_Key) };
    // units::radian_t offsetFR { frc::Preferences::GetDouble(constants::preferences::offsetFR_Key) };
    // units::radian_t offsetBL { frc::Preferences::GetDouble(constants::preferences::offsetBL_Key) };
    // units::radian_t offsetBR { frc::Preferences::GetDouble(constants::preferences::offsetBR_Key) };

    for(std::unique_ptr<SwerveModule>& swerveModule : m_swerveModules)
    {
        swerveModule->UpdateDriveController(driveP, driveI, driveD, driveFF_S, driveFF_V);
        swerveModule->UpdateTurnController(turnP, turnI, turnD, turnF, turnV, turnA);
    }

    // m_swerveModules[0]->UpdateTurnEncoderOffset(offsetFL);
    // m_swerveModules[1]->UpdateTurnEncoderOffset(offsetFR);
    // m_swerveModules[2]->UpdateTurnEncoderOffset(offsetBL);
    // m_swerveModules[3]->UpdateTurnEncoderOffset(offsetBR);
}