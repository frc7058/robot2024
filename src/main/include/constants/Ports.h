#pragma once

namespace ports 
{
    namespace drive 
    {
        namespace driveMotorCAN 
        {
            constexpr int frontLeft = 5;
            constexpr int frontRight = 2;
            constexpr int backLeft = 6;
            constexpr int backRight = 4;
        }

        namespace turnMotorCAN 
        {
            constexpr int frontLeft = 8;
            constexpr int frontRight = 1;
            constexpr int backLeft = 7;
            constexpr int backRight = 3;
        }

        namespace CANCoder 
        {
            constexpr int frontLeft = 9;
            constexpr int frontRight = 10;
            constexpr int backLeft = 11;
            constexpr int backRight = 12;
        }
    }

    namespace intake 
    {
        constexpr int intakeMotorPWM = 0;
    }

    namespace shooter 
    {
        constexpr int leftMotorCAN = 0;
        constexpr int rightMotorCAN = 0;
    }

    namespace dio 
    {
        constexpr int photoElectricSensor = 0;
    }
}