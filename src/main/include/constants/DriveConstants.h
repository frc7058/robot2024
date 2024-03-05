#pragma once 

#include <numbers>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace constants 
{
    namespace drive 
    {
        // Motor voltage limits
        constexpr units::volt_t maxDriveVoltage = 8.0_V;
        constexpr units::volt_t maxTurnVoltage = 8.0_V;
        
        // Maximum drive velocities
        constexpr units::meters_per_second_t maxDriveVelocity = 2.0_mps;
        constexpr units::radians_per_second_t maxAngularVelocity {1.5 * std::numbers::pi};

        constexpr double driveMeasurementFudgeFactor = 0.975; // To match expected real-world measurements
        constexpr double angularVelocityFudgeFactor = 1.0; // To account for lateral drift

        // Drive encoder measurement values
        constexpr uint32_t driveEncoderDepth = 8;
        constexpr uint32_t driveEncoderPeriod = 32;

        // Cosine scaling of drive motor speed
        constexpr bool enableCosineScaling = true;
        constexpr double cosineScalingExponent = 1.0;

        namespace encoderOffsets 
        {
            constexpr units::radian_t frontLeft = 1.2471263805510262_rad;
            constexpr units::radian_t frontRight = -0.6258641614573416_rad;
            constexpr units::radian_t backLeft = -1.8837284075235674_rad;
            constexpr units::radian_t backRight = 2.885417862012891_rad;
        }

        // Turn motor profiled PIDF values
        namespace turnPID  
        {
            constexpr double p = 4.5;
            constexpr double i = 0.0;
            constexpr double d = 0.0;
            constexpr double f = 0.10;

            // Maximum angular velocity (rad/s)
            constexpr units::radians_per_second_t maxVelocity {10 * std::numbers::pi}; 

            // Maximum angular acceleration (rad/s^2)
            constexpr units::radians_per_second_squared_t maxAcceleration {10 * std::numbers::pi}; 

            // constexpr units::radian_t tolerance = 0.002_rad; // ~0.1 degrees
        }

        // Drive motor PID values
        namespace drivePID 
        {
            constexpr double p = 1.0;
            constexpr double i = 0.05;
            constexpr double d = 0.0;
        }

        // Drive motor feedforward values
        namespace driveFF
        {
            constexpr auto s = 0.0_V; 
            constexpr auto v = 0.0_V * 1.0_s / 1.0_m;
            constexpr auto a = 0.0_V * 1.0_s * 1.0_s / 1.0_m;
        }

        // Heading lock PID values 
        namespace headingPID 
        {
            constexpr double p = 0.0;
            constexpr double i = 0.0;
            constexpr double d = 0.0;
        }

        // Preference keys for testing
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
    }
}