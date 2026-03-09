/**
 * ============================================================
 *  CAPTURA AUTOMÁTICA DE IMÁGENES — Freenove ESP32-S3 CAM
 * ============================================================
 *  Propósito : Recolección de dataset para detección de palomas
 *  Hardware  : Freenove ESP32-S3 CAM (16MB Flash, OV2640)
 *  Modos     : 1) Captura por intervalo de tiempo
 *              2) Captura por detección de movimiento
 *  Almacén   : Tarjeta microSD (SD_MMC)
 *  Extras    : LED indicador, contador de imágenes en SPIFFS
 * ============================================================
 *  LIBRERÍAS REQUERIDAS (Arduino IDE / Platform IO):
 *    - esp32 board package (Espressif)
 *    - FS, SD_MMC (incluidas en el paquete ESP32)
 *
 *  BOARD SETTINGS en Arduino IDE:
 *    Board       : "ESP32S3 Dev Module"
 *    Flash Size  : 16MB
 *    PSRAM       : "OPI PSRAM"
 *    Partition   : "Huge APP (3MB No OTA/1MB SPIFFS)"
 * ============================================================
 */

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"

// ============================================================
//  CONFIGURACIÓN — MODIFICA ESTOS VALORES SEGÚN TU NECESIDAD
// ============================================================

// Modo de captura: 1 = Intervalo fijo | 2 = Detección de movimiento
#define MODO_CAPTURA        1

// Intervalo entre capturas en modo 1 (milisegundos)
// 10000 = cada 10 segundos
#define INTERVALO_MS        10000

// Umbral de movimiento (0-255). Menor = más sensible.
// Ajusta según cuánto ruido/vibración tenga tu cámara.
#define UMBRAL_MOVIMIENTO   30

// Tiempo mínimo entre capturas en modo 2 (evita ráfagas)
#define COOLDOWN_MOVIMIENTO 3000

// Resolución de captura para el dataset
// Opciones: FRAMESIZE_96X96, FRAMESIZE_QQVGA(160x120),
//           FRAMESIZE_QVGA(320x240), FRAMESIZE_VGA(640x480)
// Para Edge Impulse con FOMO: FRAMESIZE_QVGA es buen balance
#define RESOLUCION          FRAMESIZE_QVGA

// Calidad JPEG (0-63, menor número = mayor calidad)
#define CALIDAD_JPEG        10

// Prefijo del nombre de archivo en la SD
// Las imágenes se guardarán como: /dataset/img_000001.jpg
#define PREFIJO_ARCHIVO     "/dataset/img_"

// LED indicador de captura (GPIO integrado en Freenove ESP32-S3)
#define PIN_LED             2

// ============================================================
//  PINES DE CÁMARA — Freenove ESP32-S3 CAM (NO MODIFICAR)
// ============================================================
#define PWDN_GPIO_NUM       -1
#define RESET_GPIO_NUM      -1
#define XCLK_GPIO_NUM       15
#define SIOD_GPIO_NUM       4
#define SIOC_GPIO_NUM       5
#define Y9_GPIO_NUM         16
#define Y8_GPIO_NUM         17
#define Y7_GPIO_NUM         18
#define Y6_GPIO_NUM         12
#define Y5_GPIO_NUM         10
#define Y4_GPIO_NUM         8
#define Y3_GPIO_NUM         9
#define Y2_GPIO_NUM         11
#define VSYNC_GPIO_NUM      6
#define HREF_GPIO_NUM       7
#define PCLK_GPIO_NUM       13

// ============================================================
//  VARIABLES GLOBALES
// ============================================================
static uint32_t contadorImagenes = 0;
static uint32_t ultimaCaptura    = 0;
static uint8_t* frameAnterior    = nullptr;
static size_t   tamanoFrame      = 0;

// ============================================================
//  INICIALIZACIÓN DE CÁMARA
// ============================================================
bool inicializarCamara() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    // Usar PSRAM si está disponible para resoluciones mayores
    if (psramFound()) {
        config.frame_size     = RESOLUCION;
        config.jpeg_quality   = CALIDAD_JPEG;
        config.fb_count       = 2;
        config.fb_location    = CAMERA_FB_IN_PSRAM;
        Serial.println("[CAM] PSRAM encontrado — calidad máxima habilitada");
    } else {
        // Sin PSRAM limitar resolución
        config.frame_size     = FRAMESIZE_QQVGA;
        config.jpeg_quality   = 12;
        config.fb_count       = 1;
        config.fb_location    = CAMERA_FB_IN_DRAM;
        Serial.println("[CAM] ADVERTENCIA: Sin PSRAM, resolución reducida");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Error al inicializar: 0x%x\n", err);
        return false;
    }

    // Ajustes de imagen para mejor calidad en exteriores
    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 0);      // -2 a 2
    s->set_contrast(s, 1);        // -2 a 2
    s->set_saturation(s, 0);      // -2 a 2
    s->set_sharpness(s, 1);       // -2 a 2
    s->set_whitebal(s, 1);        // Balance de blancos automático ON
    s->set_awb_gain(s, 1);        // AWB gain ON
    s->set_wb_mode(s, 0);         // 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home
    s->set_exposure_ctrl(s, 1);   // Exposición automática ON
    s->set_aec2(s, 1);            // AEC DSP ON
    s->set_ae_level(s, 0);        // -2 a 2
    s->set_gain_ctrl(s, 1);       // Control de ganancia ON
    s->set_agc_gain(s, 0);        // 0 a 30
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_bpc(s, 0);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_hmirror(s, 0);         // Cambiar a 1 si la imagen sale espejada
    s->set_vflip(s, 0);           // Cambiar a 1 si la imagen sale invertida

    Serial.println("[CAM] Cámara inicializada correctamente");
    return true;
}

// ============================================================
//  INICIALIZACIÓN DE SD CARD
// ============================================================
bool inicializarSD() {
    // SD_MMC usa pines internos del Freenove ESP32-S3:
    // CLK=39, CMD=38, D0=40 (modo 1-bit)
    if (!SD_MMC.begin("/sdcard", true)) {  // true = modo 1-bit (más compatible)
        Serial.println("[SD] Error: No se pudo montar la tarjeta SD");
        Serial.println("[SD] Verifica que la SD esté insertada y formateada en FAT32");
        return false;
    }

    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[SD] No se detectó tarjeta SD");
        return false;
    }

    Serial.print("[SD] Tipo de tarjeta: ");
    switch(cardType) {
        case CARD_MMC:  Serial.println("MMC"); break;
        case CARD_SD:   Serial.println("SDSC"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default:        Serial.println("DESCONOCIDO"); break;
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("[SD] Tamaño: %lluMB\n", cardSize);

    // Crear carpeta /dataset si no existe
    if (!SD_MMC.exists("/dataset")) {
        SD_MMC.mkdir("/dataset");
        Serial.println("[SD] Carpeta /dataset creada");
    }

    // Contar imágenes existentes para no sobrescribir
    File root = SD_MMC.open("/dataset");
    if (root) {
        File file = root.openNextFile();
        while (file) {
            contadorImagenes++;
            file = root.openNextFile();
        }
        root.close();
    }

    Serial.printf("[SD] Imágenes existentes: %d\n", contadorImagenes);
    Serial.printf("[SD] Espacio libre: %lluMB\n",
                  SD_MMC.totalBytes() / (1024 * 1024) -
                  SD_MMC.usedBytes() / (1024 * 1024));
    return true;
}

// ============================================================
//  GUARDAR IMAGEN EN SD
// ============================================================
bool guardarImagen(camera_fb_t* fb) {
    contadorImagenes++;
    char ruta[40];
    snprintf(ruta, sizeof(ruta), "%s%06d.jpg", PREFIJO_ARCHIVO, contadorImagenes);

    File archivo = SD_MMC.open(ruta, FILE_WRITE);
    if (!archivo) {
        Serial.printf("[SD] Error: No se pudo crear %s\n", ruta);
        contadorImagenes--;
        return false;
    }

    size_t bytesEscritos = archivo.write(fb->buf, fb->len);
    archivo.close();

    if (bytesEscritos != fb->len) {
        Serial.printf("[SD] Error: escritura incompleta (%d/%d bytes)\n",
                      bytesEscritos, fb->len);
        return false;
    }

    Serial.printf("[OK] Guardado: %s (%d bytes, %dx%d)\n",
                  ruta, fb->len, fb->width, fb->height);
    return true;
}

// ============================================================
//  DETECCIÓN DE MOVIMIENTO (comparación de frames)
//  Retorna el porcentaje de píxeles que cambiaron (0.0 - 100.0)
// ============================================================
float detectarMovimiento(camera_fb_t* fb) {
    // Necesitamos un frame anterior para comparar
    if (frameAnterior == nullptr || tamanoFrame != fb->len) {
        // Primer frame o cambió el tamaño: guardar referencia
        if (frameAnterior != nullptr) free(frameAnterior);
        frameAnterior = (uint8_t*)malloc(fb->len);
        if (frameAnterior == nullptr) return 0.0;
        memcpy(frameAnterior, fb->buf, fb->len);
        tamanoFrame = fb->len;
        return 0.0;
    }

    // Comparar píxeles (muestreo cada 4 bytes para eficiencia)
    uint32_t diferencias = 0;
    uint32_t muestras    = 0;
    for (size_t i = 0; i < fb->len; i += 4) {
        int diff = abs((int)fb->buf[i] - (int)frameAnterior[i]);
        if (diff > UMBRAL_MOVIMIENTO) diferencias++;
        muestras++;
    }

    // Actualizar frame de referencia
    memcpy(frameAnterior, fb->buf, fb->len);

    return (muestras > 0) ? (float)diferencias / muestras * 100.0f : 0.0f;
}

// ============================================================
//  PARPADEO DEL LED INDICADOR
// ============================================================
void parpadearLED(int veces, int duracionMs) {
    for (int i = 0; i < veces; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(duracionMs);
        digitalWrite(PIN_LED, LOW);
        if (i < veces - 1) delay(duracionMs);
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("  Captura de Dataset — Detección Palomas");
    Serial.println("========================================");

    // LED indicador
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // Inicializar cámara
    if (!inicializarCamara()) {
        Serial.println("[FATAL] No se pudo inicializar la cámara. Reiniciando...");
        parpadearLED(10, 100);
        ESP.restart();
    }

    // Inicializar SD
    if (!inicializarSD()) {
        Serial.println("[FATAL] No se pudo inicializar la SD. Reiniciando...");
        parpadearLED(5, 200);
        ESP.restart();
    }

    // Modo de operación
    Serial.println("\n--- CONFIGURACIÓN ACTIVA ---");
    Serial.printf("Modo     : %s\n",
                  MODO_CAPTURA == 1 ? "Intervalo fijo" : "Detección de movimiento");
    if (MODO_CAPTURA == 1) {
        Serial.printf("Intervalo: cada %d segundos\n", INTERVALO_MS / 1000);
    } else {
        Serial.printf("Umbral mov: %d | Cooldown: %d ms\n",
                      UMBRAL_MOVIMIENTO, COOLDOWN_MOVIMIENTO);
    }
    Serial.println("----------------------------\n");

    // Parpadeo de confirmación: sistema listo
    parpadearLED(3, 150);
    Serial.println("[READY] Sistema listo. Iniciando capturas...\n");

    // Capturar y descartar los primeros 2 frames
    // (la cámara necesita estabilizarse al encender)
    for (int i = 0; i < 2; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        delay(100);
    }
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {
    uint32_t ahora = millis();

    // ---- MODO 1: Captura por intervalo ----
    if (MODO_CAPTURA == 1) {
        if (ahora - ultimaCaptura >= INTERVALO_MS) {
            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("[ERROR] Fallo al obtener frame de cámara");
                delay(500);
                return;
            }

            digitalWrite(PIN_LED, HIGH);
            bool exito = guardarImagen(fb);
            esp_camera_fb_return(fb);
            digitalWrite(PIN_LED, LOW);

            if (!exito) {
                Serial.println("[WARN] Error al guardar imagen, continuando...");
            }

            ultimaCaptura = ahora;

            // Mostrar estadísticas cada 10 imágenes
            if (contadorImagenes % 10 == 0) {
                Serial.printf("[STATS] Total imágenes: %d | Espacio libre: %lluMB\n",
                              contadorImagenes,
                              (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024));
            }
        }
    }

    // ---- MODO 2: Detección de movimiento ----
    else if (MODO_CAPTURA == 2) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            delay(100);
            return;
        }

        float movimiento = detectarMovimiento(fb);

        if (movimiento > 5.0 && (ahora - ultimaCaptura >= COOLDOWN_MOVIMIENTO)) {
            Serial.printf("[MOV] Movimiento detectado: %.1f%% — Capturando...\n",
                          movimiento);

            digitalWrite(PIN_LED, HIGH);
            bool exito = guardarImagen(fb);
            digitalWrite(PIN_LED, LOW);

            if (exito) ultimaCaptura = ahora;
        }

        esp_camera_fb_return(fb);
        delay(200);  // Revisar movimiento cada 200ms
    }
}
