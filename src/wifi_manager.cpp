#include "wifi_manager.h"
#include <WiFi.h>
#include "credentials.h"

// =================================================================
// ESTADO INTERNO DEL MÓDULO
// =================================================================

static String currentSSID = SSID_FAMILIA;
static String currentPASS = PASS_FAMILIA;

// =================================================================
// IMPLEMENTACIÓN
// =================================================================

bool wifi_connect() {
    Serial.print("Conectando a WiFi ");
    WiFi.begin(currentSSID.c_str(), currentPASS.c_str());

    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(500));

        retryCount++;
        if (retryCount > 60) { // 30 segundos
            Serial.println("\nError WiFi: Reiniciando...");
            ESP.restart();
        }
    }

    Serial.println("\nWiFi Conectado.");
    return true;
}

bool wifi_switch(const char* ssid, const char* pass) {
    String ssidAnterior = currentSSID;
    String passAnterior = currentPASS;

    Serial.println("Iniciando cambio de red hacia: " + String(ssid));

    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));

    WiFi.begin(ssid, pass);

    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 15) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        Serial.print(".");
        intentos++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        currentSSID = ssid;
        currentPASS = pass;
        Serial.println("\nCambio de red exitoso. IP: " + WiFi.localIP().toString());
        return true;
    } else {
        Serial.println("\nFallo al conectar. Ejecutando ROLLBACK a red anterior...");

        WiFi.disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        WiFi.begin(ssidAnterior.c_str(), passAnterior.c_str());
        return false;
    }
}

bool wifi_reconnect() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.println("WiFi perdido, reconectando...");
    WiFi.disconnect();
    WiFi.reconnect();
    vTaskDelay(pdMS_TO_TICKS(5000));
    return false;
}

bool wifi_isConnected() {
    return WiFi.status() == WL_CONNECTED;
}
