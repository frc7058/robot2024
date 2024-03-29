// #include "lib/NavX.h"
// #include "constants/GeneralConstants.h"

// #include <chrono>
// #include <thread>
// #include <string>

// NavX::NavX()
// {
//     using clock = std::chrono::steady_clock;

//     auto start = clock::now();
//     bool timeout = false;

//     std::chrono::seconds timeoutLimit { constants::navXTimeoutSeconds };

//     while(m_navX.IsCalibrating() || !m_navX.IsConnected())
//     {
//         auto elapsed_time = clock::now() - start;
//         if(elapsed_time >= timeoutLimit)
//         {
//             const std::string error_text = "NavX could not connect/calibrate. Related functionalities will be disabled.";

//             // Add static error logging
//             fmt::print("\n{}\n", std::string('-', error_text.length()));
//             fmt::print("{}\n", error_text);
//             fmt::print("{}\n\n", std::string('-', error_text.length()));

//             timeout = true;
//             break;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds {100});
//     }

//     if(!timeout)
//     {
//         m_isAvailable = true;
//         m_navX.ZeroYaw();
//     }
// }

// AHRS& NavX::Get()
// {
//     return m_navX;
// }

// bool NavX::IsAvailable()
// {
//     return m_isAvailable && m_navX.IsConnected();
// }