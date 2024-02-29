#pragma once

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <chrono>
#include <string>

namespace constants 
{
    constexpr double pi = std::numbers::pi;
    constexpr units::radian_t pi_radians {pi};

    constexpr std::chrono::seconds navXTimeout {30};

    namespace drive
    {
        // Motor voltage limits
        constexpr units::volt_t maxDriveVoltage = 8.0_V;
        constexpr units::volt_t maxTurnVoltage = 8.0_V;
        
        // Maximum drive velocities
        constexpr units::meters_per_second_t maxDriveVelocity = 2.0_mps;
        constexpr units::radians_per_second_t maxAngularVelocity {1.5 * constants::pi};

        // Maximum drive accelerations
        // constexpr units::meters_per_second_squared_t maxDriveAcceleration = 40.0_mps_sq;
        // constexpr units::radians_per_second_squared_t maxAngularAcceleration {15.0 * constants::pi};

        // Drivebase measurements and information
        constexpr units::meter_t wheelDiameter = 0.1016_m;
        constexpr units::meter_t wheelCircumference = wheelDiameter * pi;
        constexpr units::meter_t moduleDistanceX = 0.27305_m; 
        constexpr units::meter_t moduleDistanceY = 0.27305_m; 
        constexpr units::meter_t moduleRadius = 0.38615_m;
        constexpr double driveGearRatio = 6.75;

        constexpr double driveMeasurementFudgeFactor = 1.0; // To match expected real-world measurements
        constexpr double angularVelocityFudgeFactor = 1.0; // To account for lateral drift

        // Drive encoder measurement values
        constexpr uint32_t driveEncoderDepth = 8;
        constexpr uint32_t driveEncoderPeriod = 32;

        // Drive speed cosine scaling 
        constexpr bool enableCosineScaling = true;
        constexpr double cosineScalingExponent = 1.0;

        namespace pid
        {
            // Drive motor PID values
            constexpr double drivePID_P {1.0};
            constexpr double drivePID_I {0.05};
            constexpr double drivePID_D {0.0};
            //constexpr auto drivePID_V = 40.0_mps_sq; // Maximum drive acceleration
            //constexpr auto drivePID_A = 400.0_mps_sq / 1.0_s; // Maximum drive jerk

            // Turn motor PID values
            constexpr double turnPID_P {4.5};
            constexpr double turnPID_I {0.0};
            constexpr double turnPID_D {0.0};
            constexpr double turnPID_F {0.10};
            constexpr units::radians_per_second_t turnPID_V {10 * constants::pi}; // Maximum angular velocity (rad/s)
            constexpr units::radians_per_second_squared_t turnPID_A {10 * constants::pi}; // Maximum angular acceleration (rad/s^2)
            // constexpr units::radian_t turnPIDTolerance = 0.002_rad; // ~0.1 degrees
        }

        namespace feedforward 
        {
            // Drive motor feedforward values
            constexpr auto drive_S = 0.0_V; 
            constexpr auto drive_V = 0.0_V * 1.0_s / 1.0_m;
            constexpr auto drive_A = 0.0_V * 1.0_s * 1.0_s / 1.0_m;
        }

        // CANCoder (turn encoder) offsets
        namespace encoder_offsets 
        {
            constexpr units::radian_t frontLeft = 1.2471263805510262_rad;
            constexpr units::radian_t frontRight = -0.6258641614573416_rad;
            constexpr units::radian_t backLeft = -1.8837284075235674_rad;
            constexpr units::radian_t backRight = 2.885417862012891_rad;
        }

        constexpr pathplanner::HolonomicPathFollowerConfig pathFollowerConfig(
            // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0),

            // Rotation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0),

            // Max swerve module velocity
            constants::drive::maxDriveVelocity,

            // Distance from robot center to swerve modules
            constants::drive::moduleRadius,

            // Default path replanning config
            pathplanner::ReplanningConfig()
        );
    }

    namespace intake
    {
        constexpr units::volt_t intakePower = 0.0_V;
        constexpr units::volt_t feedToOuttakePower = 0.0_V;
    }

    namespace preferences
    {
        constexpr std::string_view driveP_Key = "Drive PID P";
        constexpr std::string_view driveI_Key = "Drive PID I";
        constexpr std::string_view driveD_Key = "Drive PID D";
        constexpr std::string_view driveFF_S_Key = "Drive FF S";
        constexpr std::string_view driveFF_V_Key = "Drive FF V";
        
        constexpr std::string_view turnP_Key = "Turn PID P";
        constexpr std::string_view turnI_Key = "Turn PID I";
        constexpr std::string_view turnD_Key = "Turn PID D";
        constexpr std::string_view turnV_Key = "Turn PID V";
        constexpr std::string_view turnA_Key = "Turn PID A";
        constexpr std::string_view turnF_Key = "Turn FF";

        constexpr std::string_view offsetFL_Key = "Front Left Offset";
        constexpr std::string_view offsetFR_Key = "Front Right Offset";
        constexpr std::string_view offsetBL_Key = "Back Left Offset";
        constexpr std::string_view offsetBR_Key = "Back Right Offset";
    }

    namespace controls
    {
        constexpr double joystickDeadband = 0.08;
    }

    namespace can_ports 
    {
        namespace drive_motor
        {
            constexpr int front_left = 5;
            constexpr int front_right = 2;
            constexpr int back_left = 6;
            constexpr int back_right = 4;
        }

        namespace turn_motor
        {
            constexpr int front_left = 8;
            constexpr int front_right = 1;
            constexpr int back_left = 7;
            constexpr int back_right = 3;
        }

        namespace can_coder 
        {
            constexpr int front_left = 9;
            constexpr int front_right = 10;
            constexpr int back_left = 11;
            constexpr int back_right = 12;
        }
    }

    namespace pwm_channels
    {
        constexpr int intakeMotor = 0;
    }

    namespace dio_ports
    {
        constexpr int photoElectricSensor = 0;
    }
}