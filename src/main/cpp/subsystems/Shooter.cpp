#include "subsystems/Shooter.h"
#include "constants/ShooterConstants.h"
#include "constants/Ports.h"

#include <units/voltage.h>
#include <frc/RobotController.h>

Shooter::Shooter()
{
    fmt::print("\nInitializing Shooter...\n");

    m_feedMotor = std::make_unique<rev::CANSparkMax>(
        ports::shooter::feedMotorCAN,
        rev::CANSparkBase::MotorType::kBrushless);

    m_leftMotor = std::make_unique<rev::CANSparkMax>(
        ports::shooter::leftMotorCAN,
        rev::CANSparkBase::MotorType::kBrushless);

    m_rightMotor = std::make_unique<rev::CANSparkMax>(
        ports::shooter::rightMotorCAN,
        rev::CANSparkBase::MotorType::kBrushless);
        
    m_feedMotor->SetInverted(true);
    m_leftMotor->SetInverted(true);
    m_rightMotor->SetInverted(false);

    m_leftMotor->SetSmartCurrentLimit(60);
    m_rightMotor->SetSmartCurrentLimit(60);

    m_feedMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_leftMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_rightMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);

    // m_feedMotor->SetSmartCurrentLimit(constants::shooter::maxCurrent.value());
    // m_leftMotor->SetSmartCurrentLimit(constants::shooter::maxCurrent.value());
    // m_rightMotor->SetSmartCurrentLimit(constants::shooter::maxCurrent.value());

    // m_feedMotor->BurnFlash();
    // m_leftMotor->BurnFlash();
    // m_rightMotor->BurnFlash();

    m_leftEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_leftMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_leftEncoder->SetPositionConversionFactor(1.0 / constants::shooter::shooterGearReduction);
    m_leftEncoder->SetVelocityConversionFactor(1.0 / constants::shooter::shooterGearReduction);

    m_rightEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_rightMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_rightEncoder->SetPositionConversionFactor(1.0 / constants::shooter::shooterGearReduction);
    m_rightEncoder->SetVelocityConversionFactor(1.0 / constants::shooter::shooterGearReduction);

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
    
    // fmt::print("Left: {}          Right: {}\n", m_leftEncoder->GetVelocity(), m_rightEncoder->GetVelocity());

    // m_leftMotor->SetVoltage(leftOutput);
    // m_rightMotor->SetVoltage(rightOutput);
}

void Shooter::SetShooterVoltage(const units::volt_t voltage)
{
    // Might need to invert this.
    m_leftMotor->SetVoltage(voltage);
    m_rightMotor->SetVoltage(voltage);
}

void Shooter::SetShooterSpeed(const units::revolutions_per_minute_t speed)
{
    m_leftPID->SetSetpoint(speed.value());
    m_rightPID->SetSetpoint(speed.value());

    m_targetSpeed = speed;
}

void Shooter::StopShooter()
{
    m_leftPID->SetSetpoint(0);
    m_rightPID->SetSetpoint(0);

    m_targetSpeed = 0.0_rad_per_s;

    m_leftMotor->StopMotor();
    m_rightMotor->StopMotor();
}

units::revolutions_per_minute_t Shooter::GetLeftSpeed() const
{
    return units::revolutions_per_minute_t { m_leftEncoder->GetVelocity() };
}

units::revolutions_per_minute_t Shooter::GetRightSpeed() const
{
    return units::revolutions_per_minute_t { m_rightEncoder->GetVelocity() };
}

void Shooter::RunFeeder(const units::volt_t voltage)
{
    m_feedMotor->SetVoltage(voltage);
}

void Shooter::StopFeeder()
{
    m_feedMotor->StopMotor();
}

bool Shooter::AtSpeed() const 
{
    return m_leftPID->AtSetpoint() && m_rightPID->AtSetpoint();
}

frc2::sysid::SysIdRoutine Shooter::GetSysIdRoutine()
{
    return frc2::sysid::SysIdRoutine(
        frc2::sysid::Config(std::nullopt, std::nullopt, std::nullopt, std::nullopt),

        frc2::sysid::Mechanism(
            [this] (units::volt_t voltage)
            {
                m_leftMotor->SetVoltage(voltage);
                m_rightMotor->SetVoltage(voltage);
            },

            [this] (frc::sysid::SysIdRoutineLog* log)
            {
                units::volt_t batteryVoltage = frc::RobotController::GetBatteryVoltage();

                log->Motor("Left Motor")
                    .voltage(m_leftMotor->Get() * batteryVoltage)
                    .position(units::meter_t {m_leftEncoder->GetPosition()})
                    .velocity(units::meters_per_second_t {m_leftEncoder->GetVelocity()});

                log->Motor("Right Motor")
                    .voltage(m_rightMotor->Get() * batteryVoltage)
                    .position(units::meter_t {m_rightEncoder->GetPosition()})
                    .velocity(units::meters_per_second_t {m_rightEncoder->GetVelocity()});
            },

            this
        )
    );
}