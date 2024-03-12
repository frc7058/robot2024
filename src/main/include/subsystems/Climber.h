#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/voltage.h>
#include <memory>

class Climber : public frc2::SubsystemBase
{
public:
    Climber();

    void Set(units::volt_t voltage);
    void Stop();

private:
    // Dual motor gearbox
    std::unique_ptr<rev::CANSparkMax> m_motor1;
    std::unique_ptr<rev::CANSparkMax> m_motor2;
};