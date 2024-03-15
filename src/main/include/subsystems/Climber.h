#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/PowerDistribution.h>
#include <rev/CANSparkMax.h>
#include <units/voltage.h>
#include <memory>

class Climber : public frc2::SubsystemBase
{
public:
    Climber(frc::PowerDistribution& pdh);

    void Periodic() override;

    void SetVoltage(units::volt_t voltage);
    void Stop();

    bool CurrentLimitReached() const;

    bool ReachedMaxPosition() const;
    bool ReachedMinPosition() const;

    bool IsLocked() const;
    void Unlock();

private:
    frc::PowerDistribution& m_pdh;

    // Dual motor gearbox
    std::unique_ptr<rev::CANSparkMax> m_motor1;
    std::unique_ptr<rev::CANSparkMax> m_motor2;

    std::unique_ptr<rev::SparkRelativeEncoder> m_encoder1;
    std::unique_ptr<rev::SparkRelativeEncoder> m_encoder2;

    bool m_locked = false;
};