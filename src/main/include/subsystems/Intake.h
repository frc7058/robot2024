#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
//#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/Spark.h>
#include <units/voltage.h>
#include <memory>
#include "Constants.h"

class Intake : public frc2::SubsystemBase
{
public:
    Intake();

    //void Periodic() override;

    void RunIntake(units::volt_t voltage);
    void StopIntake();
    
    bool IsNoteDetected() const;

private:
    //std::unique_ptr<rev::CANSparkMax> m_intakeMotor;
    std::unique_ptr<frc::Spark> m_intakeMotor;
    std::unique_ptr<frc::DigitalInput> m_photoElectricSensor; 
};