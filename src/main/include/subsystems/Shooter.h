#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/filter/Debouncer.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <memory>

class Shooter : public frc2::SubsystemBase
{
public:
    Shooter();

    void Periodic() override;

    void SetShooterVoltage(const units::volt_t voltage);
    void SetShooterSpeed(const units::revolutions_per_minute_t speed);
    void StopShooter();
    units::revolutions_per_minute_t GetLeftSpeed() const;
    units::revolutions_per_minute_t GetRightSpeed() const;

    void RunFeeder(const units::volt_t voltage);
    void StopFeeder();

    bool AtSpeed() const;

    frc2::CommandPtr GetSysIdRoutine();

private:
    std::unique_ptr<rev::CANSparkMax> m_feedMotor;
    std::unique_ptr<rev::CANSparkMax> m_leftMotor;
    std::unique_ptr<rev::CANSparkMax> m_rightMotor;
    
    std::unique_ptr<rev::SparkRelativeEncoder> m_leftEncoder;
    std::unique_ptr<rev::SparkRelativeEncoder> m_rightEncoder;

    std::unique_ptr<frc::ProfiledPIDController<units::turns_per_second>> m_leftPID;
    std::unique_ptr<frc::ProfiledPIDController<units::turns_per_second>> m_rightPID;

    //std::unique_ptr<frc::SimpleMotorFeedforward<units::radians>> m_feedForward;
    std::unique_ptr<frc::SimpleMotorFeedforward<units::turns>> m_feedForward;

    //std::unique_ptr<frc::Debouncer> m_debouncer;

    std::unique_ptr<frc2::sysid::SysIdRoutine> m_sysIdRoutine;
    
    units::radians_per_second_t m_targetSpeed {0};
    bool m_atSpeed = false;
};