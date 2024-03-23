#include "subsystems/Intake.h"
#include "constants/IntakeConstants.h"
#include "constants/Ports.h"
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake()
{
    fmt::print("\nInitializing Intake...\n");

    m_intakeMotor = std::make_unique<rev::CANSparkMax>(
        ports::intake::intakeMotorCAN, 
        rev::CANSparkMax::MotorType::kBrushless);

    m_intakeMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    // m_intakeMotor->SetSmartCurrentLimit(constants::intake::maxCurrent.value());
    // m_intakeMotor->BurnFlash();
    
    m_photoElectricSensor = std::make_unique<frc::DigitalInput>(ports::dio::photoElectricSensor);

    m_debouncer = std::make_unique<frc::Debouncer>(constants::intake::noteDetectionDelay, frc::Debouncer::DebounceType::kRising);

    fmt::print("Intake Initialization complete\n\n");
}

void Intake::Periodic()
{
    // fmt::print("Intake current: {}\n", m_intakeMotor->GetOutputCurrent());
    frc::SmartDashboard::PutBoolean("Note Detected", IsNoteDetected());
}

void Intake::RunIntake(units::volt_t voltage)
{
    m_intakeMotor->SetVoltage(voltage);
}

void Intake::StopIntake()
{
    m_intakeMotor->StopMotor();
}

bool Intake::IsNoteDetected() const
{
    return m_debouncer->Calculate(m_photoElectricSensor->Get());
}