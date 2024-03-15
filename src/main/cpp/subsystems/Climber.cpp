#include "subsystems/Climber.h"
#include "constants/Ports.h"
#include "constants/ClimberConstants.h"
#include <units/current.h>

Climber::Climber(frc::PowerDistribution& pdh)
    : m_pdh(pdh)
{
    m_motor1 = std::make_unique<rev::CANSparkMax>(ports::climber::motorOneCAN, rev::CANSparkBase::MotorType::kBrushless);
    m_motor1->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_motor1->SetInverted(true);

    m_motor2 = std::make_unique<rev::CANSparkMax>(ports::climber::motorTwoCAN, rev::CANSparkBase::MotorType::kBrushless);
    m_motor2->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_motor2->SetInverted(true);

    m_encoder1 = std::make_unique<rev::SparkRelativeEncoder>(m_motor1->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_encoder1->SetPosition(0);

    m_encoder2 = std::make_unique<rev::SparkRelativeEncoder>(m_motor2->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_encoder2->SetPosition(0);
}

void Climber::Periodic()
{
    // if(CurrentLimitReached() || PositionLimitReached())
    // {
    //     m_locked = true;
    //     Stop();
    // }
}

void Climber::SetVoltage(units::volt_t voltage)
{
    if(IsLocked())
    {
        Stop();
        return;
    }

    m_motor1->SetVoltage(voltage);
    m_motor2->SetVoltage(voltage);
}

void Climber::Stop()
{
    m_motor1->StopMotor();
    m_motor2->StopMotor();
}

bool Climber::CurrentLimitReached() const
{
    units::ampere_t current1 { m_pdh.GetCurrent(ports::pdh::climbMotorOneChannel) };
    units::ampere_t current2 { m_pdh.GetCurrent(ports::pdh::climbMotorTwoChannel) };

    fmt::print("Current output: {} and {}\n", current1, current2);

    return current1 > constants::climber::currentThreshold || current2 > constants::climber::currentThreshold;
}

bool Climber::ReachedMaxPosition() const 
{
    double motorPosition1 = m_encoder1->GetPosition();
    double motorPosition2 = m_encoder2->GetPosition();

    fmt::print("Position: {} and {}\n", motorPosition1, motorPosition2);

    if(motorPosition1 > constants::climber::maxMotorPositionTicks ||
        motorPosition2 > constants::climber::maxMotorPositionTicks)
    {
        fmt::print("Maximum position reached\n");
        return true;
    }

    return false;
}

bool Climber::ReachedMinPosition() const
{
    double motorPosition1 = m_encoder1->GetPosition();
    double motorPosition2 = m_encoder2->GetPosition();

    fmt::print("Position: {} anad {}\n\n", motorPosition1, motorPosition2);

    if(motorPosition1 < constants::climber::minMotorPositionTicks ||
        motorPosition2 < constants::climber::minMotorPositionTicks)
    {
        fmt::print("Minimum position reached\n");
        return true;
    }

    return false;
}

bool Climber::IsLocked() const
{
    return m_locked;
}

void Climber::Unlock()
{
    m_locked = false;
}