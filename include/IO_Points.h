#pragma once

#include <Arduino.h>

namespace Pinout {


    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (SERVO MOTORES)
    //----------------------------------------------------------------
    namespace ServoMotors {
        constexpr uint8_t SERVO_X = 6;
        constexpr uint8_t SERVO_Y = 7;
    }

    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (LASER)
    //----------------------------------------------------------------
    namespace Laser {
        constexpr uint8_t Laser_1 = A12;
    }

    //================================================================
    // SUBSISTEMA DE ACTUACICIÓN (LEDS)
    //----------------------------------------------------------------
    namespace TiraLED {
        constexpr uint8_t LEDs = A1;
    }
} // Fin del namespace Pinout
