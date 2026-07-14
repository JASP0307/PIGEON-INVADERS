#include "calibration.h"

void guardarCalibracion(int areaIndex) {
    char namespaceName[10];
    sprintf(namespaceName, "calib%d", areaIndex);

    preferences.begin(namespaceName, false); 

    preferences.putInt("minX", g_calibMinX[areaIndex]);
    preferences.putInt("maxX", g_calibMaxX[areaIndex]);
    preferences.putInt("minY", g_calibMinY[areaIndex]);
    preferences.putInt("maxY", g_calibMaxY[areaIndex]);
    preferences.putInt("homeX", g_homeX[areaIndex]);
    preferences.putInt("homeY", g_homeY[areaIndex]);

    preferences.end();
    Serial.printf("Área %d guardada exitosamente en memoria no volátil.\n", areaIndex + 1);
}

void cargarCalibraciones() {
  char namespaceName[10];

  for (int i = 0; i < AREAS; i++) {
    sprintf(namespaceName, "calib%d", i);
    preferences.begin(namespaceName, true);
    
    g_calibMinX[i] = preferences.getInt("minX", 0);
    g_calibMaxX[i] = preferences.getInt("maxX", 100);
    g_calibMinY[i] = preferences.getInt("minY", 0);
    g_calibMaxY[i] = preferences.getInt("maxY", 100);
    
    g_homeX[i] = preferences.getInt("homeX", 90);
    g_homeY[i] = preferences.getInt("homeY", 90);
    
    preferences.end();
    
    Serial.printf("--- Área %d Cargada ---\n", i + 1);
    Serial.printf("Límites: X[%i - %i], Y[%i - %i]\n", g_calibMinX[i], g_calibMaxX[i], g_calibMinY[i], g_calibMaxY[i]);
  }
}

void cargarVelocidades() {
  preferences.begin("config", true);
  MONITORING_INTERVAL_MS = preferences.getInt("intervalMins", 5) * 60000; 
  SPEED_MS = preferences.getInt("Speed", 50); 
  preferences.end();
  
  Serial.printf("Velocidades cargadas: Intervalo %d minutos, Velocidad %d ms\n", MONITORING_INTERVAL_MS / 60000, SPEED_MS);
}

void guardarPosicionServos(int x, int y) {
  preferences.begin("servoPos", false);
  preferences.putInt("lastX", x);
  preferences.putInt("lastY", y);
  preferences.end();
}

int cargarPosicionServosX() {
  preferences.begin("servoPos", true);
  int val = preferences.getInt("lastX", 90);
  preferences.end();
  return val;
}

int cargarPosicionServosY() {
  preferences.begin("servoPos", true);
  int val = preferences.getInt("lastY", 90);
  preferences.end();
  return val;
}
