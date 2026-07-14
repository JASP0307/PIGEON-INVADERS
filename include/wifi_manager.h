#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>

// Conexión WiFi inicial (bloqueante, hasta 30s).
// Retorna true si se conectó, false si falló (reinicia el ESP).
bool wifi_connect();

// Cambia la red WiFi. Retorna true si la conexión fue exitosa.
// En caso de fallo, reconecta a la red anterior automáticamente.
bool wifi_switch(const char* ssid, const char* pass);

// Reconecta si se perdió la conexión (no bloqueante).
// Retorna true si está conectado, false si no.
bool wifi_reconnect();

// Retorna true si WiFi está conectado.
bool wifi_isConnected();

#endif // WIFI_MANAGER_H
