#include "Laser.h"

// Implementación del Constructor
Laser::Laser(uint8_t pin) {
    _pin = pin;
    // Configurar el pin como una salida digital
    pinMode(_pin, OUTPUT);
    // Por seguridad, siempre empezamos con el láser apagado.F
    // Llamamos a nuestro propio método off() para asegurar que el estado sea consistente.
    off();
}

// Implementación del método para encender el láser
void Laser::on() {
    digitalWrite(_pin, HIGH); // Pone el pin en alto voltaje (ON)
    _isOn = true;             // Actualiza nuestro estado interno
}

// Implementación del método para apagar el láser
void Laser::off() {
    digitalWrite(_pin, LOW);  // Pone el pin en bajo voltaje (OFF)
    _isOn = false;            // Actualiza nuestro estado interno
}

// Implementación del método para cambiar el estado
void Laser::toggle() {
    if (_isOn) {
        off();
    } else {
        on();
    }
}

// Implementación del método para obtener el estado
bool Laser::isOn() const {
    return _isOn;
}