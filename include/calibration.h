#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "SystemDefinitions.h"

void guardarCalibracion(int areaIndex);
void cargarCalibraciones();
void cargarVelocidades();

#endif // CALIBRATION_H
