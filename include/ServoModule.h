#ifndef SERVOMODULE_H
#define SERVOMODULE_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoModule {
public:
    // --- Miembros Públicos ---
    ServoModule(uint8_t pin);            // Constructor
    
    // Sugerencia: En ESP32 es útil permitir definir los pulsos min/max en begin
    void begin(int initialAngle = 0);   
    
    void setTarget(int angle);           // Establece el ángulo objetivo
    bool update();                       // Actualiza la posición (suavizado/delay)
    int getPosition();               // Devuelve el ángulo actual
    int getTarget();               // Devuelve el ángulo target actual

private:
    // --- Miembros Privados ---
    Servo _servo;        // La clase Servo ahora viene de ESP32Servo.h
    uint8_t _pin;        
    int _currentAngle;   
    int _targetAngle;    
    bool _isAttached;    
};

#endif // SERVOMODULE_H