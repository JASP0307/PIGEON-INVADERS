#include "ServoModule.h"

// Constructor: Ahora inicializa _isAttached a false.
ServoModule::ServoModule(uint8_t pin) 
    : _pin(pin), _currentAngle(170), _targetAngle(170), _isAttached(false) {
    // El servo empieza "dormido" (detached).
}

// begin(): Ya no activa el servo. Solo prepara las variables.
void ServoModule::begin(int initialAngle) {
    _currentAngle = constrain(initialAngle, 0, 180);
    _targetAngle = _currentAngle;
    // _servo.attach(_pin); <-- CAMBIO: Eliminamos esto. El servo no se activa al inicio.
}

// setTarget(): Ahora es el responsable de "despertar" al servo.
void ServoModule::setTarget(int angle) {
    _targetAngle = constrain(angle, 0, 180);

    // Si el servo está dormido Y necesita moverse, ¡lo despertamos!
    if (!_isAttached && _targetAngle != _currentAngle) {
        _servo.attach(_pin);
        _isAttached = true;
    }
}

// update(): Ahora es el responsable de "dormir" al servo cuando llega a su destino.
bool ServoModule::update() {
    // Si el servo no está activo, no hay nada que hacer. Salida rápida.
    if (!_isAttached) {
        return false;
    }

    if (_currentAngle < _targetAngle) {
        _currentAngle++;
        _servo.write(_currentAngle);
        return true; // Sigue en movimiento.
    } 
    else if (_currentAngle > _targetAngle) {
        _currentAngle--;
        _servo.write(_currentAngle);
        return true; // Sigue en movimiento.
    } 
    else {
        // --- Hemos llegado al destino ---
        _servo.detach();         // <-- CAMBIO CLAVE: Apagamos el servo.
        _isAttached = false;     // <-- CAMBIO CLAVE: Actualizamos el flag.
        return false;            // Movimiento finalizado.
    }
}

int ServoModule::getPosition() {
    return _currentAngle;
}

int ServoModule::getTarget() {
    return _targetAngle;
}
