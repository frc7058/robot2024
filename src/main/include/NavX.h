#pragma once

#include <AHRS.h>
#include "Constants.h"

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