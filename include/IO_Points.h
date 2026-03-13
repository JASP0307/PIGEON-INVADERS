#pragma once

#include <Arduino.h>

namespace Pinout {


    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (SERVO MOTORES)
    //----------------------------------------------------------------
    namespace ServoMotors {
        constexpr uint8_t SERVO_X = 42;
        constexpr uint8_t SERVO_Y = 21;
    }

    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (LASER)
    //----------------------------------------------------------------
    namespace Laser {
        constexpr uint8_t Laser_1 = 48;
    }
} // Fin del namespace Pinout
