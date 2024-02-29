#include "subsystems/DriveBase.h"
#include <fmt/color.h>
#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>

DriveBase::DriveBase()
{
    frc::SmartDashboard::PutBoolean("Cosine scaling", true);
    frc::SmartDashboard::PutNumber("Cosine scaling exponent", constants::drive::cosineScalingExponent);

    fmt::print("\nInitializing DriveBase...\n");

    m_swerveModules[0] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (FL)", 
        constants::can_ports::drive_motor::front_left, 
        constants::can_ports::turn_motor::front_left, 
        constants::can_ports::can_coder::front_left, 
        constants::drive::encoder_offsets::frontLeft));

    m_swerveModules[1] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (FR)", 
        constants::can_ports::drive_motor::front_right, 
        constants::can_ports::turn_motor::front_right, 
        constants::can_ports::can_coder::front_right, 
        constants::drive::encoder_offsets::frontRight));
    m_swerveModules[1]->SetDriveMotorInverted(true);

    m_swerveModules[2] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (BL)", 
        constants::can_ports::drive_motor::back_left, 
        constants::can_ports::turn_motor::back_left, 
        constants::can_ports::can_coder::back_left, 
        constants::drive::encoder_offsets::backLeft));

    m_swerveModules[3] = std::move(std::make_unique<SwerveModule>(
        "Swerve Module (BR)", 
        constants::can_ports::drive_motor::back_right, 
        constants::can_ports::turn_motor::back_right, 
        constants::can_ports::can_coder::back_right, 
        constants::drive::encoder_offsets::backRight));
    m_swerveModules[3]->SetDriveMotorInverted(true);

    if(m_navX.IsAvailable())
    {
        ZeroHeading();
    }
    else 
    {
        fmt::print("NavX is not available: Field-oriented drive and odometry are disabled.\n");
    }
     
    m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(m_kinematics, GetHeading(), GetSwerveModulePositions());

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
        constants::drive::pathFollowerConfig,

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

    if(m_navX.IsAvailable())
    {
        m_odometry->Update(m_navX.Get().GetRotation2d(), GetSwerveModulePositions());
    }
}

void DriveBase::Drive(units::meters_per_second_t velocityX, units::meters_per_second_t velocityY, units::radians_per_second_t angularVelocity, bool fieldRelative)
{
    //fmt::print("Vx: {} m/s, Vy: {} m/s, Vangular: {} rad/s\n", velocityX.value(), velocityY.value(), angularVelocity.value());

    frc::ChassisSpeeds chassisSpeeds { velocityX, velocityY, angularVelocity };

    if(fieldRelative && m_navX.IsAvailable())
    {
        frc::Rotation2d robotRotation = m_navX.Get().GetRotation2d();
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, robotRotation);
    }

    Drive(chassisSpeeds);
}

void DriveBase::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    //units::radians_per_second_t angularVelocity = chassisSpeeds.omega;
    //chassisSpeeds.omega *= constants::drive::angularVelocityFudgeFactor;
    chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, 0.02_s);
    //chassisSpeeds.omega = angularVelocity;

    wpi::array<frc::SwerveModuleState, 4> swerveModuleStates = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
    m_kinematics.DesaturateWheelSpeeds(&swerveModuleStates, constants::drive::maxDriveVelocity);

    for(int index = 0; index < m_swerveModules.size(); index++)
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

units::radian_t DriveBase::GetHeading()
{
    if(m_navX.IsAvailable())
    {
        return units::convert<units::degrees, units::radians>(units::degree_t { m_navX.Get().GetYaw() });
    }
    else 
    {
        return units::radian_t {0};
    }
}

void DriveBase::ZeroHeading()
{
    m_navX.Get().ZeroYaw();
}

frc::Pose2d DriveBase::GetPose() const
{
    return m_odometry->GetPose();
}

void DriveBase::ResetPose(frc::Pose2d pose)
{
    m_odometry->ResetPosition(GetHeading(), GetSwerveModulePositions(), pose);
}

frc::ChassisSpeeds DriveBase::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds(GetSwerveModuleStates());
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

void DriveBase::InitializePreferences()
{
    frc::Preferences::InitDouble(constants::preferences::driveP_Key, constants::drive::pid::drivePID_P);
    frc::Preferences::InitDouble(constants::preferences::driveI_Key, constants::drive::pid::drivePID_I);
    frc::Preferences::InitDouble(constants::preferences::driveD_Key, constants::drive::pid::drivePID_D);
    frc::Preferences::InitDouble(constants::preferences::driveFF_S_Key, constants::drive::feedforward::drive_S.value());
    frc::Preferences::InitDouble(constants::preferences::driveFF_V_Key, constants::drive::feedforward::drive_V.value());
    
    frc::Preferences::InitDouble(constants::preferences::turnP_Key, constants::drive::pid::turnPID_P);
    frc::Preferences::InitDouble(constants::preferences::turnI_Key, constants::drive::pid::turnPID_I);
    frc::Preferences::InitDouble(constants::preferences::turnD_Key, constants::drive::pid::turnPID_D);
    frc::Preferences::InitDouble(constants::preferences::turnF_Key, constants::drive::pid::turnPID_F);
    frc::Preferences::InitDouble(constants::preferences::turnV_Key, constants::drive::pid::turnPID_V.value());
    frc::Preferences::InitDouble(constants::preferences::turnA_Key, constants::drive::pid::turnPID_A.value());

    frc::Preferences::InitDouble(constants::preferences::offsetFL_Key, constants::drive::encoder_offsets::frontLeft.value());
    frc::Preferences::InitDouble(constants::preferences::offsetFR_Key, constants::drive::encoder_offsets::frontRight.value());
    frc::Preferences::InitDouble(constants::preferences::offsetBL_Key, constants::drive::encoder_offsets::backLeft.value());
    frc::Preferences::InitDouble(constants::preferences::offsetBR_Key, constants::drive::encoder_offsets::backRight.value());

    LoadPreferences();
}

void DriveBase::LoadPreferences()
{
    double driveP = frc::Preferences::GetDouble(constants::preferences::driveP_Key);
    double driveI = frc::Preferences::GetDouble(constants::preferences::driveI_Key);
    double driveD = frc::Preferences::GetDouble(constants::preferences::driveD_Key);
    double driveFF_S = frc::Preferences::GetDouble(constants::preferences::driveFF_S_Key);
    double driveFF_V = frc::Preferences::GetDouble(constants::preferences::driveFF_V_Key);

    double turnP = frc::Preferences::GetDouble(constants::preferences::turnP_Key);
    double turnI = frc::Preferences::GetDouble(constants::preferences::turnI_Key);
    double turnD = frc::Preferences::GetDouble(constants::preferences::turnD_Key);
    double turnF = frc::Preferences::GetDouble(constants::preferences::turnF_Key);
    units::radians_per_second_t turnV { frc::Preferences::GetDouble(constants::preferences::turnV_Key) };
    units::radians_per_second_squared_t turnA { frc::Preferences::GetDouble(constants::preferences::turnA_Key) };

    units::radian_t offsetFL { frc::Preferences::GetDouble(constants::preferences::offsetFL_Key) };
    units::radian_t offsetFR { frc::Preferences::GetDouble(constants::preferences::offsetFR_Key) };
    units::radian_t offsetBL { frc::Preferences::GetDouble(constants::preferences::offsetBL_Key) };
    units::radian_t offsetBR { frc::Preferences::GetDouble(constants::preferences::offsetBR_Key) };

    for(std::unique_ptr<SwerveModule>& swerveModule : m_swerveModules)
    {
        swerveModule->UpdateDriveController(driveP, driveI, driveD, driveFF_S, driveFF_V);
        swerveModule->UpdateTurnController(turnP, turnI, turnD, turnF, turnV, turnA);
    }

    m_swerveModules[0]->UpdateTurnEncoderOffset(offsetFL);
    m_swerveModules[1]->UpdateTurnEncoderOffset(offsetFR);
    m_swerveModules[2]->UpdateTurnEncoderOffset(offsetBL);
    m_swerveModules[3]->UpdateTurnEncoderOffset(offsetBR);
}