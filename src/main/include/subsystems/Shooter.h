#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/CANSparkMax.h>
#include <units/voltage.h>
#include <memory>
#include "Constants.h"

class Shooter : public frc2::SubsystemBase
{
public:
    Shooter();

    void Periodic() override;

    void SetSpeed(units::revolutions_per_minute_t speed);
    units::revolutions_per_minute_t GetLeftSpeed() const;
    units::revolutions_per_minute_t GetRightSpeed() const;

    bool ReachedSpeed() const;

    void Stop();

private:
    std::unique_ptr<rev::CANSparkMax> m_leftMotor;
    std::unique_ptr<rev::CANSparkMax> m_rightMotor;
    
    std::unique_ptr<rev::SparkRelativeEncoder> m_leftEncoder;
    std::unique_ptr<rev::SparkRelativeEncoder> m_rightEncoder;

    std::unique_ptr<frc::PIDController> m_leftPID;
    std::unique_ptr<frc::PIDController> m_rightPID;

    std::unique_ptr<frc::SimpleMotorFeedforward<units::radians>> m_feedForward;
    
    units::radians_per_second_t m_targetSpeed {0};
};