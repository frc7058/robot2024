#include "subsystems/Shooter.h"
#include "constants/ShooterConstants.h"
#include "constants/Ports.h"

#include <units/voltage.h>
#include <frc/RobotController.h>
#include <frc2/command/Commands.h>

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
    m_leftMotor->SetInverted(false);
    m_rightMotor->SetInverted(true);

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
    m_leftEncoder->SetAverageDepth(constants::shooter::encoderDepth);
    m_leftEncoder->SetMeasurementPeriod(constants::shooter::encoderPeriod);
    
    m_rightEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_rightMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_rightEncoder->SetAverageDepth(constants::shooter::encoderDepth);
    m_rightEncoder->SetMeasurementPeriod(constants::shooter::encoderPeriod);

    m_leftPID = std::make_unique<frc::ProfiledPIDController<units::turns_per_second>>(
        constants::shooter::pid::p,
        constants::shooter::pid::i,
        constants::shooter::pid::d,
        frc::TrapezoidProfile<units::turns_per_second>::Constraints(
            constants::shooter::pid::v,
            constants::shooter::pid::a
        ));
    m_leftPID->SetTolerance(units::turns_per_second_t {1.0});

    m_rightPID = std::make_unique<frc::ProfiledPIDController<units::turns_per_second>>(
        constants::shooter::pid::p,
        constants::shooter::pid::i,
        constants::shooter::pid::d,
        frc::TrapezoidProfile<units::turns_per_second>::Constraints(
            constants::shooter::pid::v,
            constants::shooter::pid::a
        ));
    m_rightPID->SetTolerance(units::turns_per_second_t {1.0});

    m_feedForward = std::make_unique<frc::SimpleMotorFeedforward<units::turns>>(
        constants::shooter::feedforward::s,
        constants::shooter::feedforward::v,
        constants::shooter::feedforward::a
    );

    fmt::print("Shooter Initialization complete\n\n");
}

void Shooter::Periodic()
{
    units::turns_per_second_t leftSpeed = GetLeftSpeed();
    units::turns_per_second_t rightSpeed = GetRightSpeed();

    units::volt_t leftOutputPID { m_leftPID->Calculate(leftSpeed) };
    leftOutputPID = std::clamp(leftOutputPID, -constants::shooter::pid::maxOutput, constants::shooter::pid::maxOutput);
    units::volt_t leftOutputFF = m_feedForward->Calculate(m_leftPID->GetSetpoint().position);
    units::volt_t leftOutput = leftOutputPID + leftOutputFF;
    leftOutput = std::clamp(leftOutput, -constants::shooter::maxVoltage, constants::shooter::maxVoltage);

    units::volt_t rightOutputPID { m_rightPID->Calculate(rightSpeed) };
    rightOutputPID = std::clamp(rightOutputPID, -constants::shooter::pid::maxOutput, constants::shooter::pid::maxOutput);
    units::volt_t rightOutputFF = m_feedForward->Calculate(m_rightPID->GetSetpoint().position);
    units::volt_t rightOutput = rightOutputPID + rightOutputFF;
    rightOutput = std::clamp(rightOutput, -constants::shooter::maxVoltage, constants::shooter::maxVoltage);

    
    if(std::abs(m_leftEncoder->GetVelocity()) + std::abs(m_rightEncoder->GetVelocity()) > 10)
    {
        fmt::print("Left: {}       Right: {}\n", leftSpeed, rightSpeed);
    }
 
    // m_leftMotor->SetVoltage(leftOutput);
    // m_rightMotor->SetVoltage(rightOutput);
}

void Shooter::SetShooterVoltage(const units::volt_t voltage)
{
    m_leftMotor->SetVoltage(voltage);
    m_rightMotor->SetVoltage(voltage);
}

void Shooter::SetShooterSpeed(const units::revolutions_per_minute_t speed)
{
    m_leftPID->SetGoal(speed);
    m_rightPID->SetGoal(speed);

    m_targetSpeed = speed;
}

void Shooter::StopShooter()
{
    m_leftPID->SetGoal(units::turns_per_second_t {0.0});
    m_rightPID->SetGoal(units::turns_per_second_t {0.0});

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
    bool atSpeed = m_leftPID->AtGoal() && m_rightPID->AtGoal();
    if(atSpeed)
    {
        fmt::print("Goal: {} and {}\n", m_leftPID->GetGoal().position, m_rightPID->GetGoal().position);
        fmt::print("At goal: {} and {}\n", GetLeftSpeed(), GetRightSpeed());
    }

    return atSpeed;
}

frc2::CommandPtr Shooter::GetSysIdRoutine()
{
    m_sysIdRoutine = std::make_unique<frc2::sysid::SysIdRoutine>(
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
                    .position(units::turn_t {m_leftEncoder->GetPosition()})
                    .velocity(units::turns_per_second_t {m_leftEncoder->GetVelocity() / 60.0});

                log->Motor("Right Motor")
                    .voltage(m_rightMotor->Get() * batteryVoltage)
                    .position(units::turn_t {m_rightEncoder->GetPosition()})
                    .velocity(units::turns_per_second_t {m_rightEncoder->GetVelocity() / 60.0});
            },

            this
        )
    );

    return m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kForward)
        .AndThen(frc2::cmd::Wait(5.0_s))
        .AndThen(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kReverse))
        .AndThen(frc2::cmd::Wait(5.0_s))
        .AndThen(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kForward))
        .AndThen(frc2::cmd::Wait(5.0_s))
        .AndThen(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kReverse));
}