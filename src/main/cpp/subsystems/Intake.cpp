#include "subsystems/Intake.h"
#include "constants/IntakeConstants.h"
#include "constants/Ports.h"

Intake::Intake()
{
    fmt::print("\nInitializing Intake...\n");

    //m_intakeMotor = std::make_unique<rev::CANSparkMax>(0, rev::CANSparkMax::MotorType::kBrushless);
    m_intakeMotor = std::make_unique<frc::Spark>(ports::intake::intakeMotorPWM);
    m_photoElectricSensor = std::make_unique<frc::DigitalInput>(ports::dio::photoElectricSensor);

    fmt::print("Intake Initialization complete\n\n");
}

void Intake::RunIntake(units::volt_t voltage)
{
    m_intakeMotor->SetVoltage(voltage);
}

void Intake::StopIntake()
{
    m_intakeMotor->SetVoltage(0.0_V);
    m_intakeMotor->StopMotor();
}

bool Intake::IsNoteDetected() const
{
    return m_photoElectricSensor->Get();
}