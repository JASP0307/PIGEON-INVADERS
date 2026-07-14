#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <Arduino.h>
#include "esp_camera.h"

// Inicializa la cámara OV2640 con la configuración del Freenove ESP32-S3 CAM.
// Retorna true si la inicialización fue exitosa.
bool camera_init();

// Captura una foto JPEG. Retorna el framebuffer o NULL si falla.
// El caller debe llamar camera_releaseFrame() cuando termine de usarlo.
camera_fb_t* camera_capture();

// Libera el framebuffer capturado. CRÍTICO para evitar memory leaks en PSRAM.
void camera_releaseFrame(camera_fb_t* fb);

// Envía una foto JPEG a Telegram vía HTTPS multipart POST.
// Retorna true si la API de Telegram respondió ok:true.
bool camera_sendToTelegram(const char* chatId, const char* botToken,
                           uint8_t* photoBuf, size_t photoLen,
                           const String& caption);

#endif // CAMERA_MANAGER_H
