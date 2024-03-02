#include "subsystems/Shooter.h"

Shooter::Shooter()
{
    fmt::print("\nInitializing Shooter...\n");

    m_leftMotor = std::make_unique<rev::CANSparkMax>(
        constants::can_ports::shooter_motor::left, 
        rev::CANSparkBase::MotorType::kBrushless);

    m_rightMotor = std::make_unique<rev::CANSparkMax>(
        constants::can_ports::shooter_motor::right, 
        rev::CANSparkBase::MotorType::kBrushless);

    m_leftEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_leftMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_rightEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_rightMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));

    m_leftPID = std::make_unique<frc::PIDController>(
        constants::shooter::pid::p,
        constants::shooter::pid::i,
        constants::shooter::pid::d
    );

    m_rightPID = std::make_unique<frc::PIDController>(
        constants::shooter::pid::p,
        constants::shooter::pid::i,
        constants::shooter::pid::d
    );

    m_feedForward = std::make_unique<frc::SimpleMotorFeedforward<units::radians>>(
        constants::shooter::feedforward::s,
        constants::shooter::feedforward::v,
        constants::shooter::feedforward::a
    );

    fmt::print("Shooter Initialization complete\n\n");
}

void Shooter::Periodic()
{
    units::radians_per_second_t leftSpeed = units::convert<units::revolutions_per_minute, units::radians_per_second>(GetLeftRPM());

    units::volt_t feedforwardOutput = m_feedForward->Calculate(m_targetSpeed);

    units::volt_t leftOutput { m_leftPID->Calculate(leftRPM.value()) };
    leftOutput += feedforwardOutput;

    units::volt_t rightOutput { m_rightPID->Calculate(leftRPM.value()) };
    leftOutput += feedforwardOutput;
}

units::revolutions_per_minute_t Shooter::GetLeftRPM() const
{
    return units::revolutions_per_minute_t { m_leftEncoder->GetVelocity() };
}

units::revolutions_per_minute_t Shooter::GetRightSpeed() const
{
    return units::revolutions_per_minute_t { m_rightEncoder->GetVelocity() };
}