#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "SystemDefinitions.h"

void guardarCalibracion(int areaIndex);
void cargarCalibraciones();
void cargarVelocidades();
void guardarPosicionServos(int x, int y);
int cargarPosicionServosX();
int cargarPosicionServosY();

#endif // CALIBRATION_H
