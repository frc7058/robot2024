#include "subsystems/LEDLights.h"

LEDLights::LEDLights()
{
    m_led.SetLength(constants::LED::length);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void LEDLights::Periodic()
{
    m_led.SetData(m_ledBuffer);
    
    m_tickCounter++;
}

void LEDLights::SetSolidRGB(int r, int g, int b)
{
    for(size_t index = 0; index < constants::LED::length; index++)
    {
        m_ledBuffer[index].SetRGB(r, g, b);
    }
}

void LEDLights::RainbowPattern()
{
    int firstPixelHue = (m_tickCounter * 3) % 180;

    for(size_t index = 0; index < constants::LED::length; index++)
    {
        int hue = (firstPixelHue + static_cast<int>(index * 180.0 / constants::LED::length)) % 180;
        m_ledBuffer[index].SetRGB(hue, 255, 128);
    }
}

void LEDLights::BlueAndGold()
{
    for(size_t index = 0; index < constants::LED::length; index++)
    {
        frc::Color8Bit color = constants::LED::colors::blue;

        if((m_tickCounter + index) % 2 == 0)
            color = constants::LED::colors::gold;

        m_ledBuffer[index].SetLED(color);
    }
}