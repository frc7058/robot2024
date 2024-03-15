#include "lib/SwerveModule.h"
#include "lib/Util.h"
#include "constants/GeneralConstants.h"
#include "constants/DriveConstants.h"
#include <units/math.h>

SwerveModule::SwerveModule(std::string name, int driveMotorCanID, int turnMotorCanID, int canCoderCanID, units::radian_t canCoderOffset)
    : m_name(name), m_encoderOffset(canCoderOffset)
{
    fmt::print("Initializing Swerve Module {}", name);

    m_driveMotor = std::make_unique<rev::CANSparkMax>(driveMotorCanID, rev::CANSparkMax::MotorType::kBrushless);
    m_driveMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_turnMotor = std::make_unique<rev::CANSparkMax>(turnMotorCanID, rev::CANSparkMax::MotorType::kBrushless);
    m_turnMotor->SetInverted(true);
    m_turnMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_turnEncoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(canCoderCanID);

    m_driveEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_driveMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    m_driveEncoder->SetAverageDepth(constants::drive::driveEncoderDepth);
    m_driveEncoder->SetMeasurementPeriod(constants::drive::driveEncoderPeriod);

    units::meter_t positionConversionFactor = constants::drive::wheelCircumference / (constants::drive::driveGearRatio) * constants::drive::driveMeasurementFudgeFactor;
    m_driveEncoder->SetPositionConversionFactor(positionConversionFactor.value());
    
    units::meter_t velocityConversionFactor = positionConversionFactor / 60.0;
    m_driveEncoder->SetVelocityConversionFactor(velocityConversionFactor.value());

    m_drivePID = std::make_unique<frc::PIDController>(
        constants::drive::drivePID::p,
        constants::drive::drivePID::i,
        constants::drive::drivePID::d);
    m_drivePID->SetSetpoint(0);

    m_turnPID = std::make_unique<frc::ProfiledPIDController<units::radians>>(
        constants::drive::turnPID::p,
        constants::drive::turnPID::i,
        constants::drive::turnPID::d,
        frc::TrapezoidProfile<units::radians>::Constraints {
           constants::drive::turnPID::maxVelocity,
           constants::drive::turnPID::maxAcceleration
        });
    m_turnPID->SetGoal(0_rad);
    m_turnPID->EnableContinuousInput(-constants::piRadians, constants::piRadians);
    m_turnPID->SetTolerance(constants::drive::turnPID::tolerance);
    m_turnPID_F = constants::drive::turnPID::f;

    m_driveFeedForward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        constants::drive::driveFF::s,
        constants::drive::driveFF::v,
        constants::drive::driveFF::a);

    // Check if the motors or encoders have detected faults
    bool faults_detected = ((m_driveMotor->GetStickyFaults() & m_turnMotor->GetStickyFaults() & m_turnEncoder->GetStickyFaultField().GetValue()) == 0);
    bool okay = m_driveMotor && m_turnMotor && m_turnEncoder && m_driveEncoder && m_drivePID && m_turnPID && m_driveFeedForward;

    if(okay)
    {
        if(!faults_detected)
        {
            fmt::print(" [OK]\n");
        }
        else 
        {
            fmt::print(" [FAULTS DETECTED]\n");
        }
    }
    else
    {
        fmt::print(" [ERROR]\n");
    }
}

void SwerveModule::Periodic()
{
    units::meters_per_second_t velocity = GetDriveVelocity();
    units::volt_t driveOutput { m_drivePID->Calculate(velocity.value()) };

    units::meters_per_second_t targetVelocity { m_drivePID->GetSetpoint() };
    driveOutput += m_driveFeedForward->Calculate(targetVelocity);
    driveOutput = std::clamp(driveOutput, -constants::drive::maxDriveVoltage, constants::drive::maxDriveVoltage);

    m_driveMotor->SetVoltage(driveOutput);
    
    /*
    //fmt::print("kS: {}, kV: {}\n", m_driveFeedForward->kS, m_driveFeedForward->kV);

    if(m_name == "Swerve Module (FL)")
        fmt::print("Target velocity: {} m/s, velocity: {} m/s, voltage: {}V\n", targetVelocity.value(), velocity.value(), driveOutput.value());
    */

    units::radian_t angle = GetTurnAngle();
    units::volt_t turnOutput { m_turnPID->Calculate(angle) };

    if(m_turnPID->AtGoal())
    {
        m_turnMotor->StopMotor();
    }
    else 
    {
        turnOutput += units::volt_t { m_turnPID_F * util::sign(turnOutput.value()) };
        turnOutput = std::clamp(turnOutput, -constants::drive::maxTurnVoltage, constants::drive::maxTurnVoltage);
        m_turnMotor->SetVoltage(turnOutput);
    }
}   

units::radian_t SwerveModule::GetTurnAngle() const
{
    units::radian_t angle { m_turnEncoder->GetAbsolutePosition().GetValue() };
    return frc::AngleModulus(angle + m_encoderOffset);
}

units::radian_t SwerveModule::GetTargetTurnAngle() const
{
    units::radian_t angle = m_turnPID->GetGoal().position;
    return frc::AngleModulus(angle);
}

units::meter_t SwerveModule::GetDriveDistance() const
{
    return units::meter_t { m_driveEncoder->GetPosition() };
}

units::meters_per_second_t SwerveModule::GetDriveVelocity() const
{
    return units::meters_per_second_t { m_driveEncoder->GetVelocity() };
}

frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    return frc::SwerveModulePosition { GetDriveDistance(), GetTurnAngle() };
}

frc::SwerveModuleState SwerveModule::GetState() const
{
    return frc::SwerveModuleState { GetDriveVelocity(), GetTurnAngle() };
}

void SwerveModule::SetTurnMotorInverted(bool inverted)
{
    m_turnMotor->SetInverted(inverted);
}

void SwerveModule::SetDriveMotorInverted(bool inverted)
{
    m_driveMotor->SetInverted(inverted);
}

void SwerveModule::SetTurnAngle(units::radian_t angle)
{
    m_turnPID->SetGoal(frc::AngleModulus(angle));
}

void SwerveModule::SetDriveVelocity(units::meters_per_second_t velocity)
{
    m_drivePID->SetSetpoint(velocity.value());
}

void SwerveModule::SetTargetState(frc::SwerveModuleState state)
{
    SetTurnAngle(state.angle.Radians());
    SetDriveVelocity(state.speed);
}

void SwerveModule::UpdateTurnController(double p, double i, double d, double f, units::radians_per_second_t v, units::radians_per_second_squared_t a)
{
    m_turnPID->SetPID(p, i, d);
    m_turnPID->SetConstraints(frc::TrapezoidProfile<units::radians>::Constraints { v, a });
    m_turnPID_F = f;
}

void SwerveModule::UpdateDriveController(double p, double i, double d, double ff_S, double ff_V)
{
    m_drivePID->SetPID(p, i, d);
    m_driveFeedForward.reset(new frc::SimpleMotorFeedforward<units::meters>(
        ff_S * 1.0_V,
        ff_V * 1.0_V * 1.0_s / 1.0_m,
        constants::drive::driveFF::a
    ));
}

void SwerveModule::UpdateTurnEncoderOffset(units::radian_t offset)
{
    m_encoderOffset = offset;
}