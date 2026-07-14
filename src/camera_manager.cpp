#include "camera_manager.h"
#include <WiFiClientSecure.h>
#include "SystemDefinitions.h"

// =================================================================
// INICIALIZACIÓN DE CÁMARA
// =================================================================

bool camera_init() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.grab_mode = CAMERA_GRAB_LATEST;

    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        Serial.println("PSRAM detectada. Usando buffers dobles.");
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 1;
        Serial.println("ADVERTENCIA: No se detectó PSRAM.");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error al inicializar la camara: 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_brightness(s, 1);

    Serial.println("Camara inicializada correctamente");
    return true;
}

// =================================================================
// CAPTURA DE FOTO
// =================================================================

camera_fb_t* camera_capture() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Error: Fallo al capturar la imagen de la camara.");
        return NULL;
    }
    Serial.printf("Foto capturada con exito. Tamano: %u bytes\n", fb->len);
    return fb;
}

void camera_releaseFrame(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

// =================================================================
// ENVÍO A TELEGRAM VÍA HTTPS MULTIPART POST
// =================================================================

bool camera_sendToTelegram(const char* chatId, const char* botToken,
                           uint8_t* photoBuf, size_t photoLen,
                           const String& caption) {
    WiFiClientSecure client;
    client.setInsecure();

    const char* host = "api.telegram.org";
    const int port = 443;

    Serial.printf("RAM libre: %d bytes\n", ESP.getFreeHeap());
    delay(100);
    Serial.println("Conectando a Telegram para enviar captura...");

    if (!client.connect(host, port)) {
        Serial.println("Error: No se pudo conectar a api.telegram.org");
        return false;
    }

    String boundary = "----PigeonBoundary123456789";

    String head = "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    head += String(chatId) + "\r\n";

    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"caption\"\r\n\r\n";
    head += caption + "\r\n";

    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"photo\"; filename=\"intruder.jpg\"\r\n";
    head += "Content-Type: image/jpeg\r\n\r\n";

    String tail = "\r\n--" + boundary + "--\r\n";

    uint32_t totalLen = head.length() + photoLen + tail.length();

    client.println("POST /bot" + String(botToken) + "/sendPhoto HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=" + boundary);
    client.println("Connection: close");
    client.println();

    client.write((uint8_t*)head.c_str(), head.length());

    size_t chunkSize = 2048;
    for (size_t i = 0; i < photoLen; i += chunkSize) {
        size_t currentChunk = (photoLen - i < chunkSize) ? (photoLen - i) : chunkSize;
        client.write(photoBuf + i, currentChunk);
        yield();
    }

    client.write((uint8_t*)tail.c_str(), tail.length());
    client.flush();

    String response = "";
    uint32_t timeout = millis();
    while (client.connected() && millis() - timeout < 5000) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (line == "\r") {
                break;
            }
        }
    }

    if (client.available()) {
        response = client.readStringUntil('\n');
    }

    client.stop();

    if (response.indexOf("\"ok\":true") != -1) {
        Serial.println("Foto enviada con exito!");
        return true;
    } else {
        Serial.println("Error de la API de Telegram o respuesta vacia: " + response);
        return false;
    }
}
