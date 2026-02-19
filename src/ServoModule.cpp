#include "ServoModule.h"

// Constructor: Ahora inicializa _isAttached a false.
ServoModule::ServoModule(uint8_t pin) 
    : _pin(pin), _currentAngle(0), _targetAngle(0), _isAttached(false) {
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
    // Si no se ha inicializado el attach, salimos
    if (!_isAttached) {
        //Opcional: Auto-recuperación si quieres que se conecte solo al llamar update
        _servo.attach(_pin); 
        _isAttached = true;
        return false; 
    }

    // Si ya estamos en el objetivo, NO HACEMOS NADA.
    // No escribimos, no calculamos, simplemente salimos.
    if (_currentAngle == _targetAngle) {
        return false; 
    }
    
    if (_currentAngle < _targetAngle) {
        _currentAngle++;
        _servo.write(_currentAngle);
        return true; 
    } 
    else if (_currentAngle > _targetAngle) {
        _currentAngle--;
        _servo.write(_currentAngle);
        return true; 
    } 
    else {
        // --- HEMOS LLEGADO ---
        
        // ERROR ANTERIOR: _servo.detach();  <-- ¡ESTO CAUSABA EL TEMBLOR!
        // ERROR ANTERIOR: _isAttached = false;
        
        // Mantenemos el servo conectado para que tenga fuerza y no se caiga.
        return false; 
    }
}

int ServoModule::getPosition() {
    return _currentAngle;
}

int ServoModule::getTarget() {
    return _targetAngle;
}
