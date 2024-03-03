#include "subsystems/Intake.h"
#include "Constants.h"

Intake::Intake()
{
    fmt::print("\nInitializing Intake...\n");

    //m_intakeMotor = std::make_unique<rev::CANSparkMax>(0, rev::CANSparkMax::MotorType::kBrushless);
    m_intakeMotor = std::make_unique<frc::Spark>(constants::pwm_channels::intakeMotor);
    m_photoElectricSensor = std::make_unique<frc::DigitalInput>(constants::dio_ports::photoElectricSensor);

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