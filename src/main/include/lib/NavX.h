#pragma once

#include <AHRS.h>

class NavX
{
public:
    NavX();

    AHRS& Get();
    bool IsAvailable() const;

private:
    AHRS m_navX {frc::SPI::Port::kMXP};
    bool m_isAvailable {false};
};