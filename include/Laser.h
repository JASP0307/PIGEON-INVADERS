#ifndef LASER_H
#define LASER_H

#include <Arduino.h>

class Laser {
public:
    // --- Funciones Públicas ---

    // Constructor: Se llama al crear un objeto.
    // Requiere el número del pin al que está conectado el láser.
    Laser(uint8_t pin);

    // Enciende el láser
    void on();

    // Apaga el láser
    void off();

    // Cambia el estado del láser (si está encendido lo apaga, y viceversa)
    void toggle();

    // Devuelve 'true' si el láser está encendido, 'false' si está apagado.
    bool isOn() const;

private:
    // --- Variables Privadas ---

    // El pin digital al que está conectado el láser.
    uint8_t _pin;

    // Almacena el estado actual del láser para no tener que leer el pin cada vez.
    bool _isOn;
};

#endif // LASER_H