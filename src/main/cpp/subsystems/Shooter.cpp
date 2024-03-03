#include "subsystems/Shooter.h"
#include "Constants.h"

#include <units/voltage.h>

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
    m_leftPID->SetTolerance(constants::shooter::tolerance.value());

    m_rightPID = std::make_unique<frc::PIDController>(
        constants::shooter::pid::p,
        constants::shooter::pid::i,
        constants::shooter::pid::d
    );
    m_rightPID->SetTolerance(constants::shooter::tolerance.value());

    m_feedForward = std::make_unique<frc::SimpleMotorFeedforward<units::radians>>(
        constants::shooter::feedforward::s,
        constants::shooter::feedforward::v,
        constants::shooter::feedforward::a
    );

    fmt::print("Shooter Initialization complete\n\n");
}

void Shooter::Periodic()
{
    units::revolutions_per_minute_t leftRPM = GetLeftSpeed();
    units::revolutions_per_minute_t rightRPM = GetRightSpeed();

    units::volt_t feedforwardOutput = m_feedForward->Calculate(m_targetSpeed);

    units::volt_t leftOutput { m_leftPID->Calculate(leftRPM.value()) };
    leftOutput += feedforwardOutput;
    leftOutput = std::clamp(leftOutput, -constants::shooter::maxVoltage, constants::shooter::maxVoltage);

    units::volt_t rightOutput { m_rightPID->Calculate(rightRPM.value()) };
    rightOutput += feedforwardOutput;
    rightOutput = std::clamp(leftOutput, -constants::shooter::maxVoltage, constants::shooter::maxVoltage);
    
    m_leftMotor->SetVoltage(leftOutput);
    m_rightMotor->SetVoltage(rightOutput);
}

void Shooter::SetSpeed(units::revolutions_per_minute_t speed)
{
    m_leftPID->SetSetpoint(speed.value());
    m_rightPID->SetSetpoint(speed.value());

    m_targetSpeed = units::convert<units::revolutions_per_minute, units::radians_per_second>(speed);
}

units::revolutions_per_minute_t Shooter::GetLeftSpeed() const
{
    return units::revolutions_per_minute_t { m_leftEncoder->GetVelocity() };
}

units::revolutions_per_minute_t Shooter::GetRightSpeed() const
{
    return units::revolutions_per_minute_t { m_rightEncoder->GetVelocity() };
}

bool Shooter::ReachedSpeed() const 
{
    return m_leftPID->AtSetpoint() && m_rightPID->AtSetpoint();
}

void Shooter::Stop()
{
    m_leftPID->SetSetpoint(0);
    m_rightPID->SetSetpoint(0);

    m_targetSpeed = units::radians_per_second_t {0};

    m_leftMotor->StopMotor();
    m_rightMotor->StopMotor();
}