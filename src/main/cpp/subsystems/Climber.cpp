#include "subsystems/Climber.h"
#include "constants/Ports.h"

Climber::Climber()
{
    m_motor1 = std::make_unique<rev::CANSparkMax>(ports::climber::motorOneCAN, rev::CANSparkBase::MotorType::kBrushless);
    m_motor1->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_motor2 = std::make_unique<rev::CANSparkMax>(ports::climber::motorTwoCAN, rev::CANSparkBase::MotorType::kBrushless);
    m_motor2->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}

void Climber::Set(units::volt_t voltage)
{
    m_motor1->SetVoltage(voltage);
    m_motor2->SetVoltage(voltage);
}

void Climber::Stop()
{
    m_motor1->StopMotor();
    m_motor2->StopMotor();
}