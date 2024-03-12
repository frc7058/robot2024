#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <array>
#include "constants/Ports.h"
#include "constants/GeneralConstants.h"

class LEDLights : public frc2::SubsystemBase
{
public:
    LEDLights();

    void Periodic() override;
    
    void SetSolidRGB(int r, int g, int b);

    void RainbowPattern();
    void BlueAndGold();

private:
    frc::AddressableLED m_led {ports::pwm::LEDLights};
    std::array<frc::AddressableLED::LEDData, constants::LED::length> m_ledBuffer;

    size_t m_tickCounter = 0;
};