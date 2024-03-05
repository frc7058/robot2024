#include "subsystems/Intake.h"
#include "constants/IntakeConstants.h"
#include "constants/Ports.h"

Intake::Intake()
{
    fmt::print("\nInitializing Intake...\n");

    m_intakeMotor = std::make_unique<rev::CANSparkMax>(
        ports::intake::intakeMotorCAN, 
        rev::CANSparkMax::MotorType::kBrushless);
    m_intakeMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    //m_photoElectricSensor = std::make_unique<frc::DigitalInput>(ports::dio::photoElectricSensor);

    fmt::print("Intake Initialization complete\n\n");
}

void Intake::RunIntake(units::volt_t voltage)
{
    m_intakeMotor->SetVoltage(voltage);
}

void Intake::StopIntake()
{
    m_intakeMotor->SetVoltage(0.0_V);
    //m_intakeMotor->StopMotor();
}

bool Intake::IsNoteDetected() const
{
    return true;
    //return m_photoElectricSensor->Get();
}