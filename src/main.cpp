#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Preferences.h>
#include "esp_camera.h"

#include "ServoModule.h"
#include "IO_Points.h"
#include "Laser.h"
#include "SystemDefinitions.h"

#define ZIGZAG_STEP_SIZE 1.0f // Cuánto avanza en el eje secundario
#define TARGET_TOLERANCE 0.1f  // Margen de error para decir "llegué"

// Redes conocidas
const char* SSID_FAMILIA = "FamiliaSuárez";
const char* PASS_FAMILIA = "RigobertoSuarez";

const char* SSID_CLARO = "CLAROV6RCY";
const char* PASS_CLARO = "48575443AFC7839E";

// Variables de estado de red (inician con tu red principal)
String currentSSID = SSID_FAMILIA;
String currentPASS = PASS_FAMILIA;

// Estructura para pasar el "paquete de evidencia"
// Credenciales Telegram
// Obtén esto creando un bot con @BotFather en Telegram
#define BOT_TOKEN "8378042870:AAG3z-YlLjb98I--cp8aATP29ybx_0LAfKg" 
// Tu ID numérico (puedes obtenerlo con el bot @myidbot) para seguridad
#define CHAT_ID_PERMITIDO "8210739066"

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// Intervalo de chequeo (en ms) para no saturar la red ni la API
const unsigned long BOT_MTBS = 1000; 
unsigned long bot_lasttime = 0;


// Handles globales
QueueHandle_t fsmQueue;
QueueHandle_t manualControlQueue;

SemaphoreHandle_t stateMutex;

ServoModule SERVO_X(Pinout::ServoMotors::SERVO_X);
ServoModule SERVO_Y(Pinout::ServoMotors::SERVO_Y);

Laser Laser_01(Pinout::Laser::Laser_1);

// Definir un Handle para el timer de FreeRTOS
TimerHandle_t monitoringTimer;
TimerHandle_t updateIdleLogicTimer;

TaskHandle_t xHandleServos;

// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskTelegram(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);

// Funciones auxiliares
void servos_init();
bool isWithinOperatingHours();
bool initCamera();
void startPattern(PatternType type, int min_x, int max_x, int min_y, int max_y);
void stopPattern(int x_home, int y_home);
void updatePatternLogic();
void monitoringTimerCallback(TimerHandle_t xTimer);
void updateIdleLogicTimerCallback(TimerHandle_t xTimer);
void enviarTecladoCalibracion(String chat_id);
void enviarMenu(String chat_id);
void procesarMovimientoManual(const String &text, const String &chat_id);
void procesarComandos(const String &text, const String &chat_id);
void guardarHorario(int index, int startMins, int endMins);
void procesarMensajeTelegram(String text, String chat_id);
void stopPattern(int x_home, int y_home);
void cargarHorarios();
void cargarCalibraciones();
void cargarVelocidades();


SystemState getState();
void setState(SystemState newState);

void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando sistema...");

    // Crear mutex para proteger estado
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL) {
    Serial.println("Error: No se pudo crear mutex de estado.");
    while (1);
    }

    // Crear cola FSM
    fsmQueue = xQueueCreate(10, sizeof(FSMEvent));
    if (fsmQueue == NULL) {
    Serial.println("Error: No se pudo crear la cola FSM.");
    while (1);
    }

    // Crear cola FSM
    manualControlQueue = xQueueCreate(5, sizeof(ManualPosCmd));
    if (manualControlQueue == NULL) {
    Serial.println("Error: No se pudo crear la cola manualControlQueue.");
    while (1);
    }

    // Crear tareas
    xTaskCreatePinnedToCore(TaskTelegram,               "Telegram",     10240, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(TaskComms,                  "Comms",        2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskFSM,                    "FSM",          10240, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskServoControl,           "ServoControl", 4096, NULL, 3, NULL, 1);

    Serial.println("Tareas FreeRTOS creadas");
}

void loop() {}

// Funciones thread-safe para estado
SystemState getState() {
    SystemState state;
    if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    state = currentState;
    xSemaphoreGive(stateMutex);
    }
    return state;
}

void setState(SystemState newState) {
    if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    currentState = newState;
    xSemaphoreGive(stateMutex);
    }
}

String SystemStateToString(SystemState state) {
    switch(state) {

    case STATE_INITIALIZING: return "STATE_INITIALIZING";
    case STATE_IDLE: return "STATE_IDLE";
    case STATE_MONITORING: return "STATE_MONITORING";
    case STATE_ATTACKING: return "STATE_ATTACKING";
    case STATE_CALIB_SET_LL: return "STATE_CALIB_SET_LL";
    case STATE_CALIB_SET_UR: return "STATE_CALIB_SET_UR";
    case STATE_CALIB_PREVIEW: return "STATE_CALIB_PREVIEW";
    default: return "UNKNOWN";
    }
}

void servos_init() {
    SERVO_X.begin();
    SERVO_Y.begin();
    Serial.println("Servos inicializados.");
}

bool initCamera() {
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

    // FORMATO DE IMAGEN
    // JPEG es obligatorio para enviarlo directamente a Telegram.
    config.pixel_format = PIXFORMAT_JPEG; 
    
    // TAMAÑO Y CALIDAD
    // Para algoritmos de visión artificial y Telegram, VGA o SVGA es el punto dulce.
    // Resoluciones más altas (UXGA) saturarán la memoria y harán muy lento el procesamiento FSM.
    if(psramFound()){
        config.frame_size = FRAMESIZE_VGA; // 640x480: Ideal para procesar y enviar rápido
        config.jpeg_quality = 12;          // 0-63, menor número = mayor calidad
        config.fb_count = 2;               // Usa 2 buffers para capturar más rápido
        Serial.println("PSRAM detectada. Usando buffers dobles.");
    } else {
        config.frame_size = FRAMESIZE_QVGA; // 320x240
        config.jpeg_quality = 15;
        config.fb_count = 1;
        Serial.println("ADVERTENCIA: No se detectó PSRAM.");
    }

    // Inicializar la cámara
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Error al inicializar la cámara: 0x%x\n", err);
        return false;
    }

    // Opcional: Ajustes del sensor (brillo, contraste, volteo)
    // Útil si la cámara va a estar apuntando hacia afuera desde el balcón y hay mucho sol
    sensor_t * s = esp_camera_sensor_get();
    s->set_vflip(s, 1);   // Voltear verticalmente si la montas de cabeza
    s->set_hmirror(s, 1); // Espejo horizontal
    s->set_brightness(s, 1); // Subir un poco el brillo (-2 a 2)
    
    Serial.println("✅ Cámara inicializada correctamente");
    return true;
}

// Función para iniciar un patrón (llamar desde la FSM Principal)
void startPattern(PatternType type, int min_x, int max_x, int min_y, int max_y) {
    patCtx.currentType = type;
    patCtx.minX = min_x; patCtx.maxX = max_x;
    patCtx.minY = min_y; patCtx.maxY = max_y;
    
    // Configuración inicial
    patCtx.stepIndex = 0;
    patCtx.active = true;
    
    // Punto de partida siempre es el límite inferior izquierdo (o Home)
    SERVO_X.setTarget(min_x);
    SERVO_Y.setTarget(min_y);
}

// Función para iniciar un patrón (llamar desde la FSM Principal)
void stopPattern(int x_home, int y_home) {
    patCtx.active = false;
    SERVO_X.setTarget(x_home);
    SERVO_Y.setTarget(y_home);
}

// Verifica si los servos están en posición
bool hasReachedTarget() {
    float dist01 = abs(SERVO_X.getPosition() - SERVO_X.getTarget());
    float dist02 = abs(SERVO_Y.getPosition() - SERVO_Y.getTarget());
    return (dist01 < TARGET_TOLERANCE && dist02 < TARGET_TOLERANCE);
}

void updatePatternLogic() {
    if (!patCtx.active) return;
    
    // Solo calculamos el siguiente paso si ya llegamos al anterior
    if (hasReachedTarget()) {
        float nextX, nextY;
        switch (patCtx.currentType) {
            
            // --- PATRÓN 1: RECUADRO (PREVIEW) ---
            case PATTERN_RECTANGLE_PREVIEW:
                // Recorre las 4 esquinas: (min,min) -> (max,min) -> (max,max) -> (min,max) -> (min,min)
                switch (patCtx.stepIndex) {
                    case 0: SERVO_X.setTarget(patCtx.maxX); SERVO_Y.setTarget(patCtx.minY); break; // Derecha, Abajo
                    case 1: SERVO_X.setTarget(patCtx.maxX); SERVO_Y.setTarget(patCtx.maxY); break; // Derecha, Arriba
                    case 2: SERVO_X.setTarget(patCtx.minX); SERVO_Y.setTarget(patCtx.maxY); break; // Izquierda, Arriba
                    case 3: SERVO_X.setTarget(patCtx.minX); SERVO_Y.setTarget(patCtx.minY); break; // Izquierda, Abajo (Cierre)
                    case 4: 
                        patCtx.active = false; // Fin del patrón
                        // AVISAR A LA FSM PRINCIPAL QUE TERMINAMOS
                        FSMEvent e = EVENT_PREVIEW_DONE;
                        xQueueSend(fsmQueue, &e, 0);
                        return; 
                }
                patCtx.stepIndex++;
                break;

            // --- PATRÓN 2: ZIGZAG HORIZONTAL ---
            case PATTERN_ZIGZAG_HORIZ:
                // Algoritmo: Mover X a un extremo, bajar Y un poco, Mover X al otro extremo...
                // stepIndex aquí representará las líneas verticales avanzadas
                nextY = patCtx.minY + (patCtx.stepIndex * ZIGZAG_STEP_SIZE);
                
                if (nextY > patCtx.maxY) {
                  SERVO_X.setTarget(patCtx.minX);
                  SERVO_Y.setTarget(patCtx.minY);
                  patCtx.active = false;

                  patCtx.areaPhase++; 

                  if (patCtx.areaPhase < patCtx.totalAreas) {
                      
                      int siguienteArea = patCtx.targetArea[patCtx.areaPhase];
                      
                      Serial.printf("Área %d completa, iniciando Área %d...\n", patCtx.areaPhase, patCtx.areaPhase + 1);
                      
                      startPattern(patCtx.currentType,   // mismo patrón
                          g_calibMinX[siguienteArea],
                          g_calibMaxX[siguienteArea],
                          g_calibMinY[siguienteArea],
                          g_calibMaxY[siguienteArea]
                      );
                      
                  } else {
                      Serial.println("Todas las áreas asignadas fueron atacadas. Ataque completo.");
                      FSMEvent e = EVENT_ATTACK_COMPLETE;
                      xQueueSend(fsmQueue, &e, 0);
                  }
                  
              } else {
                  float targetX = (patCtx.stepIndex % 2 == 0) ? patCtx.maxX : patCtx.minX;
                  SERVO_X.setTarget(targetX);
                  SERVO_Y.setTarget(nextY);
                  patCtx.stepIndex++;
              }
              break;
            // --- PATRÓN 3: ZIGZAG VERTICAL ---
            case PATTERN_ZIGZAG_VERT:
              // Similar al horizontal pero invirtiendo ejes
              nextX = patCtx.minX + (patCtx.stepIndex * ZIGZAG_STEP_SIZE);
              
              if (nextX > patCtx.maxX) {
                  SERVO_X.setTarget(patCtx.minX);
                  SERVO_Y.setTarget(patCtx.minY);
                  patCtx.active = false;

                  patCtx.areaPhase++; 

                  if (patCtx.areaPhase < patCtx.totalAreas) {
                      
                      int siguienteArea = patCtx.targetArea[patCtx.areaPhase];
                      
                      Serial.printf("Área %d completa, iniciando Área %d...\n", patCtx.areaPhase, patCtx.areaPhase + 1);
                      
                      startPattern(patCtx.currentType,   // mismo patrón
                          g_calibMinX[siguienteArea],
                          g_calibMaxX[siguienteArea],
                          g_calibMinY[siguienteArea],
                          g_calibMaxY[siguienteArea]
                      );
                      
                  } else {
                      Serial.println("Todas las áreas asignadas fueron atacadas. Ataque completo.");
                      FSMEvent e = EVENT_ATTACK_COMPLETE;
                      xQueueSend(fsmQueue, &e, 0);
                  }
                  
              } else {
                  float targetY = (patCtx.stepIndex % 2 == 0) ? patCtx.maxY : patCtx.minY;
                  SERVO_X.setTarget(nextX);
                  SERVO_Y.setTarget(targetY);
                  patCtx.stepIndex++;
              }
              break;

            default:
                Serial.println("Tipo de patrón desconocido");
                 patCtx.active = false;
                 break;
        }
    }
}

// Callback del Timer (se ejecuta cuando expiran los 5 mins)
void monitoringTimerCallback(TimerHandle_t xTimer) {
    FSMEvent e = EVENT_TIMER_EXPIRED;
    // Enviar a la cola desde el contexto del timer (o ISR si fuera hardware)
    xQueueSend(fsmQueue, &e, 0); 
}
// Callback del Timer para lógica de inactividad (chequeo de horas)
void updateIdleLogicTimerCallback(TimerHandle_t xTimer) {
    if (!isWithinOperatingHours() && getState() == STATE_MONITORING) {
        FSMEvent e = EVENT_STOP_COMMAND;
        xQueueSend(fsmQueue, &e, 0); 
    }else if (isWithinOperatingHours() && getState() == STATE_IDLE) {
        FSMEvent e = EVENT_START_COMMAND;
        xQueueSend(fsmQueue, &e, 0); 
    }

    Serial.println("Chequeo de horas realizado.");
}

void enviarTecladoCalibracion(String chat_id) {
    // Construimos el teclado en formato JSON
    String keyboardJson = "[";
    // Fila 1: Arriba
    keyboardJson += "[{\"text\":\"⏫\", \"callback_data\":\"TILT_UP_FAST\"}],";
    keyboardJson += "[{\"text\":\"🔼\", \"callback_data\":\"TILT_UP\"}],";
    // Fila 2: Izquierda, Guardar, Derecha
    keyboardJson += "[{\"text\":\"⏪\", \"callback_data\":\"PAN_LEFT_FAST\"},";
    keyboardJson += "{\"text\":\"◀️\", \"callback_data\":\"PAN_LEFT\"},";
    keyboardJson += "{\"text\":\"💾\", \"callback_data\":\"SAVE_LIMIT\"},";
    keyboardJson += "{\"text\":\"▶️\", \"callback_data\":\"PAN_RIGHT\"},";
    keyboardJson += "{\"text\":\"⏩\", \"callback_data\":\"PAN_RIGHT_FAST\"}],";
    // Fila 3: Abajo
    keyboardJson += "[{\"text\":\"🔽\", \"callback_data\":\"TILT_DOWN\"}],";
    keyboardJson += "[{\"text\":\"⏬\", \"callback_data\":\"TILT_DOWN_FAST\"}]";
    keyboardJson += "]";

    bot.sendMessageWithInlineKeyboard(chat_id, "Modo Calibración Activado. Usa las flechas para mover el láser:", "", keyboardJson);
}

void enviarMenu(String chat_id) {
  // Construimos el teclado en formato JSON
  
    String keyboardJson = "[";

    keyboardJson += "[\"🟢 START\", \"🔴 STOP\"],";
    keyboardJson += "[\"📸 FOTO\", \"🕊️ SIMULAR\"],";
    keyboardJson += "[\"⚙️ CALIB 1\", \"⚙️ CALIB 2\"],";
    keyboardJson += "[\"⚙️ CALIB 3\", \"⚙️ CALIB 4\"],";
    keyboardJson += "[\"👁️ VER 1\", \"👁️ VER 2\"],";
    keyboardJson += "[\"👁️ VER 3\", \"👁️ VER 4\"],";
    keyboardJson += "[\"🌐 CLARO\", \"🏠 FAMILIA\"],";
    keyboardJson += "[\"📊 STATUS\", \"❓ HELP\"]";
    keyboardJson += "]";

  // Parámetros extra: 
  // resize_keyboard = true (adapta el tamaño a los botones)
  // one_time_keyboard = false (el teclado se queda fijo en pantalla)
  // selective = false
    bot.sendMessageWithReplyKeyboard(chat_id, "Panel de Control Pigeon Invaders:", "", keyboardJson, true, false, false);
  // bot.sendMessageWithInlineKeyboard(chat_id, "Menú Principal:", "", keyboardJson);
}

void procesarMovimientoManual(const String &text, const String &chat_id)
{
    ManualPosCmd cmd;
    bool movimientoValido = true;

    if (text == "TILT_DOWN") {
        currentTilt -= stepSize;
        cmd.cmdType = 1;   // Eje Y
        cmd.value = currentTilt;
    }
    else if (text == "TILT_DOWN_FAST") {
        currentTilt -= stepSizefast;
        cmd.cmdType = 1;   // Eje Y
        cmd.value = currentTilt;
    }
    else if (text == "TILT_UP") {
        currentTilt += stepSize;
        cmd.cmdType = 1;   // Eje Y
        cmd.value = currentTilt;
    }
    else if (text == "TILT_UP_FAST") {
        currentTilt += stepSizefast;
        cmd.cmdType = 1;   // Eje Y
        cmd.value = currentTilt;
    }
    else if (text == "PAN_LEFT") {
        currentPan += stepSize;
        cmd.cmdType = 0;   // Eje X
        cmd.value = currentPan;
    }
    else if (text == "PAN_LEFT_FAST") {
        currentPan += stepSizefast;
        cmd.cmdType = 0;   // Eje X
        cmd.value = currentPan;
    }
    else if (text == "PAN_RIGHT") {
        currentPan -= stepSize;
        cmd.cmdType = 0;   // Eje X
        cmd.value = currentPan;
    }
    else if (text == "PAN_RIGHT_FAST") {
        currentPan -= stepSizefast;
        cmd.cmdType = 0;   // Eje X
        cmd.value = currentPan;
    }
    else if (text == "SAVE_LIMIT") {
        FSMEvent e = EVENT_CONFIRM_POINT;
        xQueueSend(fsmQueue, &e, 0);

        bot.sendMessage(chat_id,
            "Límite guardado para el área " + String(AREA_ACTUAL+1) + ".");
        return;  // No continuar
    }
    else {
        movimientoValido = false;
    }

    if (!movimientoValido) return;

    // Enviar a la cola sin bloquear
    
    if (xQueueSend(manualControlQueue, &cmd, 0) == pdPASS) {
    }
    else {
        bot.sendMessage(chat_id, "⚠ Cola ocupada. Intente nuevamente.", "");
    }
}

void intentarCambioWiFi(String nuevoSSID, String nuevoPASS) {
    String ssidAnterior = currentSSID;
    String passAnterior = currentPASS;

    Serial.println("Iniciando cambio de red hacia: " + nuevoSSID);
    
    // 1. Desconectar la red actual
    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(500)); // Pequeña pausa para estabilizar la radio

    // 2. Intentar nueva conexión
    WiFi.begin(nuevoSSID.c_str(), nuevoPASS.c_str());

    int intentos = 0;
    // Esperar hasta 15 segundos (15 * 1000ms)
    while (WiFi.status() != WL_CONNECTED && intentos < 15) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        Serial.print(".");
        intentos++;
    }

    // 3. Evaluar el resultado
    if (WiFi.status() == WL_CONNECTED) {
        // ¡Éxito! Actualizamos las variables globales de forma permanente
        currentSSID = nuevoSSID;
        currentPASS = nuevoPASS;
        Serial.println("\n✅ Cambio de red exitoso. IP: " + WiFi.localIP().toString());
        
        // Avisar a Telegram del éxito
        bot.sendMessage(CHAT_ID_PERMITIDO, "✅ Conectado exitosamente a la red: " + currentSSID, "");
    } else {
        // ¡Fallo! Ejecutar Rollback a la red segura
        Serial.println("\n❌ Fallo al conectar. Ejecutando ROLLBACK a red anterior...");
        
        WiFi.disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        WiFi.begin(ssidAnterior.c_str(), passAnterior.c_str());
        
        // Opcional: Podrías poner otro while aquí para asegurar que reconecta,
        // pero tu TaskTelegram ya tiene lógica de reconexión automática en su bucle infinito.
    }
}

// Función para guardar (llamar cuando Telegram reciba el comando)
void guardarHorario(int index, int startMins, int endMins) {
    preferences.begin("config", false);

    // Generamos claves dinámicas según el índice
    char keyStart[5];
    char keyEnd[5];
    sprintf(keyStart, "st%d", index);
    sprintf(keyEnd, "en%d", index);

    preferences.putInt(keyStart, startMins);
    preferences.putInt(keyEnd, endMins);
    preferences.end();

    // Actualizamos el arreglo en memoria RAM
    horariosMonitor[index].startMins = startMins;
    horariosMonitor[index].endMins = endMins;

    Serial.printf("Horario %d actualizado: %d a %d\n", index + 1, startMins, endMins);
}

// Función para guardar (llamar cuando Telegram reciba el comando)
void guardarIntervalo(int mins) {
    preferences.begin("config", false);
    preferences.putInt("intervalMins", mins);
    preferences.end();

    MONITORING_INTERVAL_MS = mins * 60000; 
    Serial.printf("Intervalo actualizado: %d minutos\n", mins);
}

// Función para guardar (llamar cuando Telegram reciba el comando)
void guardarSpeed(int ms) {
    preferences.begin("config", false);
    preferences.putInt("Speed", ms);
    preferences.end();

    SPEED_MS = ms; 
    Serial.printf("Velocidad actualizada: %d milisegundos\n", ms);
}

void procesarComandos(const String &text, const String &chat_id) {
    
    // Usamos indexOf para buscar la palabra clave sin importar los emojis
    if (text.indexOf("START") >= 0) {
        FSMEvent e = EVENT_START_COMMAND;
        xQueueSend(fsmQueue, &e, 0);
        bot.sendMessage(chat_id, "Sistema ARMADO y en MONITORING.", "Markdown");
    }
    else if (text.indexOf("STOP") >= 0) {
        FSMEvent e = EVENT_STOP_COMMAND;
        xQueueSend(fsmQueue, &e, 0);
        bot.sendMessage(chat_id, "Sistema DETENIDO.", "");
    }
    else if (text.indexOf("CALIB 1") >= 0) { // Actualiza según el texto del botón
        AREA_ACTUAL = 0;
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("CALIB 2") >= 0) {
        AREA_ACTUAL = 1; 
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("CALIB 3") >= 0) {
        AREA_ACTUAL = 2; 
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("CALIB 4") >= 0) {
        AREA_ACTUAL = 3; 
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("FOTO") >= 0) {
        FSMEvent e = EVENT_TAKE_PICTURE;
        xQueueSend(fsmQueue, &e, 0);
        bot.sendMessage(chat_id, "Capturando foto...", "");
    }
    else if (text.indexOf("SIMULAR") >= 0) {
        FSMEvent e = EVENT_PIGEON_DETECTED;
        xQueueSend(fsmQueue, &e, 0);
        bot.sendMessage(chat_id, "Simulando paloma en el área de vigilancia...", "");
    }
    else if (text.indexOf("CLARO") >= 0) {
        intentarCambioWiFi(SSID_CLARO, PASS_CLARO);
        bot.sendMessage(chat_id, "Intentando conectar a la red CLARO...", "");
    }
    else if (text.indexOf("FAMILIA") >= 0) {
        intentarCambioWiFi(SSID_FAMILIA, PASS_FAMILIA);
        bot.sendMessage(chat_id, "Intentando conectar a la red FAMILIA...", "");
    }
    else if (text.indexOf("STATUS") >= 0) {
        SystemState state = getState();
        String estadoStr = SystemStateToString(state);
        
        // 1. Iniciamos el mensaje con el estado general
        String respuesta = "📊 Estado actual del sistema: " + estadoStr + "\n\n";
        respuesta += "🕒 Horarios de Monitoreo:\n";
        
        // 2. Recorremos los 3 horarios para agregarlos al mensaje
        for (int i = 0; i < NUM_HORARIOS; i++) {
            int start = horariosMonitor[i].startMins;
            int end = horariosMonitor[i].endMins;
            
            respuesta += "  [" + String(i + 1) + "] "; // Muestra [1], [2] o [3]
            
            // Verificamos si el horario está desactivado
            if (start == 0 && end == 0) {
                respuesta += "Desactivado\n";
            } else {
                int startHH = start / 60;
                int startMM = start % 60;
                int endHH = end / 60;
                int endMM = end % 60;
                
                // Usamos sprintf para forzar 2 dígitos (ej. 08:05 en vez de 8:5)
                char buffer[20];
                sprintf(buffer, "%02d:%02d - %02d:%02d\n", startHH, startMM, endHH, endMM);
                respuesta += String(buffer);
            }
        }
        
        // 3. Agregamos el resto de los parámetros de configuración
        respuesta += "\n⚙️ Configuración Extra:\n";
        respuesta += "  • Intervalo: " + String(MONITORING_INTERVAL_MS / 60000) + " minutos\n";
        respuesta += "  • Velocidad: " + String(SPEED_MS) + " milisegundos";
        
        // 4. Enviamos el mensaje estructurado
        bot.sendMessage(chat_id, respuesta, "");
    }
    else if (text.indexOf("HELP") >= 0) {
        String ayuda = "❓ *Ayuda - Comandos Disponibles:*\n\n";
        ayuda += "🟢 *START* - Arma el sistema y comienza vigilancia.\n";
        ayuda += "🔴 *STOP* - Detiene toda actividad y vuelve a Idle.\n";
        ayuda += "📸 *FOTO* - Toma una foto instantánea del área vigilada.\n";
        ayuda += "🕊️ *SIMULAR* - Simula la detección de una paloma para pruebas.\n";
        ayuda += "⚙️ *CALIB 1-4* - Entra al modo calibración para el área 1 a 4.\n";
        ayuda += "👁️ *VER 1-4* - Muestra un video en vivo del área 1 a 4 (si implementado).\n";
        ayuda += "🌐 *CLARO* - Cambia la conexión Wi-Fi a la red CLARO.\n";
        ayuda += "🏠 *FAMILIA* - Cambia la conexión Wi-Fi a la red FAMILIA.\n";
        ayuda += "📊 *STATUS* - Muestra el estado actual del sistema.\n\n";
        ayuda += "Para comandos de calibración, usa los botones de flecha para mover el láser y el botón 💾 para guardar los límites.";
        
        bot.sendMessage(chat_id, ayuda, "Markdown");
    }
    else if (text.indexOf("VER 1") >= 0) {
        AREA_ACTUAL = 0;
        if (g_calibMinX[0] == 0 && g_calibMaxX[0] == 0) {
            bot.sendMessage(chat_id, "Área 1 no calibrada aún.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW; 
            xQueueSend(fsmQueue, &e, 0);
            bot.sendMessage(chat_id, "Mostrando Área 1...", "");
        }
    }
    else if (text.indexOf("VER 2") >= 0) {
        AREA_ACTUAL = 1;
        if (g_calibMinX[1] == 0 && g_calibMaxX[1] == 0) {
            bot.sendMessage(chat_id, "Área 2 no calibrada aún.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW; 
            xQueueSend(fsmQueue, &e, 0);
            bot.sendMessage(chat_id, "Mostrando Área 2...", "");
        }
    }
    else if (text.indexOf("VER 3") >= 0) {  
        AREA_ACTUAL = 2;
        if (g_calibMinX[2] == 0 && g_calibMaxX[2] == 0) {
            bot.sendMessage(chat_id, "Área 3 no calibrada aún.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW; 
            xQueueSend(fsmQueue, &e, 0);
            bot.sendMessage(chat_id, "Mostrando Área 3...", "");
        }
    }
    else if (text.indexOf("VER 4") >= 0) {
        AREA_ACTUAL = 3;
        if (g_calibMinX[3] == 0 && g_calibMaxX[3] == 0) {
            bot.sendMessage(chat_id, "Área 4 no calibrada aún.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW; 
            xQueueSend(fsmQueue, &e, 0);
            bot.sendMessage(chat_id, "Mostrando Área 4...", "");
        }
    }else if (text.startsWith("/horario")) {
        int id, hInicio, mInicio, hFin, mFin;

        // sscanf ahora busca 5 variables: el ID (1,2 o 3) y las horas/minutos
        if (sscanf(text.c_str(), "/horario %d %d:%d %d:%d", &id, &hInicio, &mInicio, &hFin, &mFin) == 5) {
            
            // Validamos que el ID sea correcto
            if (id < 1 || id > NUM_HORARIOS) {
                bot.sendMessage(chat_id, "⚠️ El ID del horario debe ser 1, 2 o 3.", "");
                return; 
            }

            int index = id - 1; // Ajustamos a índice de arreglo (0, 1 o 2)
            int startMins = (hInicio * 60) + mInicio;
            int endMins = (hFin * 60) + mFin;
            
            // Llamamos a la función modular
            guardarHorario(index, startMins, endMins);
            
            String respuesta = "✅ Horario " + String(id) + " de vigilancia actualizado:\n";
            respuesta += "Inicio: " + String(hInicio) + ":" + (mInicio < 10 ? "0" : "") + String(mInicio) + "\n";
            respuesta += "Fin: " + String(hFin) + ":" + (mFin < 10 ? "0" : "") + String(mFin);
            
            bot.sendMessage(chat_id, respuesta, "");
            
        } else {
            bot.sendMessage(chat_id, "⚠️ Formato incorrecto. Usa: /horario ID HH:MM HH:MM (ej. /horario 1 07:00 18:00)", "");
        }
    }else if (text.startsWith("/mins")) {
        int intervalmins = 2;
        
        // Convertimos el String a const char* para usar sscanf
        // sscanf buscará exactamente el patrón de números separados por dos puntos
        if (sscanf(text.c_str(), "/mins %d", &intervalmins) == 1) {
        guardarIntervalo(intervalmins);
        bot.sendMessage(chat_id, "✅ Intervalo de vigilancia actualizado.", "");
        } else {
        bot.sendMessage(chat_id, "⚠️ Formato incorrecto. Usa: /mins <minutos> (ej. /mins 5)", "");
        }
    }else if (text.startsWith("/speed")) {
        int velocidad_MS = 50;
        
        // Convertimos el String a const char* para usar sscanf
        // sscanf buscará exactamente el patrón de números separados por dos puntos
        if (sscanf(text.c_str(), "/speed %d", &velocidad_MS) == 1) {
        guardarSpeed(velocidad_MS);
        bot.sendMessage(chat_id, "✅ Velocidad de movimiento actualizada.", "");
        } else {
        bot.sendMessage(chat_id, "⚠️ Formato incorrecto. Usa: /speed <segundos> (ej. /speed 5)", "");
        }
    }
     else {
        // Si no es un comando específico, lo tratamos como posible movimiento manual
        procesarMovimientoManual(text, chat_id);
    }
}

void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    String text = bot.messages[i].text;
    String type = bot.messages[i].type;

    // Seguridad: Ignorar mensajes de extraños
    if (chat_id != CHAT_ID_PERMITIDO) {
        bot.sendMessage(chat_id, "Acceso denegado.", "");
        continue;
    }

    // 1. MANEJAR BOTONES INLINE (Mando de Calibración)
    if (type == "callback_query") {
        
        // Confirmar a Telegram que recibimos el clic (quita el icono de carga del botón)
        bot.answerCallbackQuery(bot.messages[i].query_id);

        // Si estamos en estado de calibración, enviamos el comando a la función manual
        if (getState() == STATE_CALIB_SET_LL || getState() == STATE_CALIB_SET_UR) {
            procesarMovimientoManual(text, chat_id);
        } else {
            // (Opcional) Mensaje si presionan un botón de calibración viejo fuera de tiempo
            bot.sendMessage(chat_id, "El modo calibración no está activo.", "");
        }
    } 
    
    // 2. MANEJAR TEXTO NORMAL Y REPLY KEYBOARD (Menú Principal)
    else if (type == "message") {
      
        if (text == "/start" || text == "/menu") {
            enviarMenu(chat_id);
        } else {
            // Le pasamos los comandos (ej. "🟢 START", "⚙️ CALIB 1") a tu procesador
            procesarComandos(text, chat_id);
        }
    }
  }
}

bool enviarFotoTelegram(String chat_id, uint8_t *photoBuf, size_t photoLen, String caption) {
    WiFiClientSecure client;

    client.setInsecure(); 

    const char* host = "api.telegram.org";
    const int port = 443;
    Serial.printf("RAM libre: %d bytes\n", ESP.getFreeHeap());
    // Darle un breve respiro al procesador antes de iniciar TLS
    delay(100); 
    Serial.println("Conectando a Telegram para enviar captura...");

    if (!client.connect(host, port)) {
        Serial.println("❌ Error: No se pudo conectar a api.telegram.org");
        return false;
    }

    // Construir las fronteras (boundaries) de la petición Multipart
    String boundary = "----PigeonBoundary123456789";
    
    // Cabecera del payload multipart
    String head = "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    head += chat_id + "\r\n";
    
    // Campo caption
    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"caption\"\r\n\r\n";
    head += caption + "\r\n";
    
    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"photo\"; filename=\"intruder.jpg\"\r\n";
    head += "Content-Type: image/jpeg\r\n\r\n";

    // Cierre del payload multipart
    String tail = "\r\n--" + boundary + "--\r\n";

    // Calcular la longitud total exacta de la petición HTTP
    uint32_t totalLen = head.length() + photoLen + tail.length();

    // 1. Enviar HTTP Headers
    client.println("POST /bot" + String(BOT_TOKEN) + "/sendPhoto HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=" + boundary);
    client.println("Connection: close"); // Decirle al servidor que cierre al terminar
    client.println(); // Fin de headers

    // 2. Enviar el inicio del Body usando write (más seguro para la longitud exacta)
    client.write((uint8_t*)head.c_str(), head.length());
    
    // 3. Enviar la FOTO en bloques (chunks)
    size_t chunkSize = 2048; // Subido a 2KB para acelerar el envío si los buffers lo permiten
    for (size_t i = 0; i < photoLen; i += chunkSize) {
        size_t currentChunk = (photoLen - i < chunkSize) ? (photoLen - i) : chunkSize;
        client.write(photoBuf + i, currentChunk);
        
        // Optimización: yield() es más amigable con el core de red que vTaskDelay en este punto
        yield(); 
    }
    
    // 4. Enviar el cierre del Body
    client.write((uint8_t*)tail.c_str(), tail.length());
    client.flush(); // Asegurar que todo se mandó por el chip de Wi-Fi

    // 5. Leer la respuesta de Telegram
    String response = "";
    // Esperar un máximo de 5 segundos a que responda el servidor
    uint32_t timeout = millis();
    while (client.connected() && millis() - timeout < 5000) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (line == "\r") {
                break; // Fin de headers de respuesta
            }
        }
    }
    
    if (client.available()) {
        response = client.readStringUntil('\n'); // Capturar el JSON de respuesta
    }
    
    client.stop();

    if (response.indexOf("\"ok\":true") != -1) {
        Serial.println("✅ ¡Foto enviada con éxito!");
        return true;
    } else {
        Serial.println("❌ Error de la API de Telegram o respuesta vacía: " + response);
        return false;
    }
}

void guardarCalibracion(int areaIndex) {
    // Crea el nombre del espacio de memoria dinámicamente (calib0 o calib1)
    char namespaceName[10];
    sprintf(namespaceName, "calib%d", areaIndex);

    // Abre el espacio correspondiente al área
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

// Función para cargar (llamar en el setup)
void cargarHorarios() {
  preferences.begin("config", true); // true significa modo "solo lectura"

  for (int i = 0; i < NUM_HORARIOS; i++) {
    // Generamos las mismas claves dinámicas que usamos al guardar
    char keyStart[5];
    char keyEnd[5];
    sprintf(keyStart, "st%d", i);
    sprintf(keyEnd, "en%d", i);

    // Definimos los valores por defecto: 
    // El Horario 1 (índice 0) será de 07:00 a 18:00 por defecto.
    // Los Horarios 2 y 3 (índices 1 y 2) estarán en 0 por defecto (desactivados).
    int defaultStart = (i == 0) ? 420 : 0;
    int defaultEnd = (i == 0) ? 1080 : 0;

    // Leemos de la memoria flash y guardamos directo en el arreglo global
    horariosMonitor[i].startMins = preferences.getInt(keyStart, defaultStart);
    horariosMonitor[i].endMins = preferences.getInt(keyEnd, defaultEnd);
    
    // Imprimimos por el monitor serial para confirmar qué se cargó al arrancar
    Serial.printf("Boot - Horario %d cargado: %d a %d minutos\n", i + 1, horariosMonitor[i].startMins, horariosMonitor[i].endMins);
  }

  preferences.end();
}

bool isWithinOperatingHours() {
  struct tm timeinfo;
  
  // Por seguridad (para no disparar láseres a las 3 AM), si falla, retornamos false.
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Error: No se pudo obtener la hora para validar el horario.");
    return false; 
  }

  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;
  int currentMins = (currentHour * 60) + currentMinute;
  
  // Opcional: Puedes comentar este Serial.printf si satura mucho el monitor serial
  // Serial.printf("Hora actual: %02d:%02d (%d minutos desde medianoche)\n", currentHour, currentMinute, currentMins);

  // Iteramos sobre los 3 horarios posibles
  for (int i = 0; i < NUM_HORARIOS; i++) {
    int start = horariosMonitor[i].startMins;
    int end = horariosMonitor[i].endMins;
    
    // Si el horario no está configurado (0 a 0), lo ignoramos para evitar falsos positivos
    if (start == 0 && end == 0) continue; 

    bool enHorario = false;

    // Evaluamos el rango actual del bucle
    if (start <= end) {
      // Horario diurno normal (ej. 07:00 a 18:00)
      enHorario = (currentMins >= start && currentMins <= end);
    } else {
      // Horario nocturno que cruza la medianoche (ej. 22:00 a 06:00)
      enHorario = (currentMins >= start || currentMins <= end);
    }

    // Si coincide con este rango, retornamos true inmediatamente (ahorra procesamiento)
    if (enHorario) {
      return true;
    }
  }

  // Si termina de revisar los 3 horarios y en ninguno coincidió, estamos fuera de servicio
  return false; 
}

void capturarYEnviarFoto(String mensajeTelegram) {
  camera_fb_t * fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("❌ Error: Fallo al capturar la imagen de la cámara.");
    bot.sendMessage(CHAT_ID_PERMITIDO, "⚠️ Fallo en la cámara al intentar capturar el área.", "");
    return; // Salimos de la función temprano
  } 
  
  Serial.printf("📸 Foto capturada con éxito. Tamaño: %u bytes\n", fb->len);
  
  struct tm timeinfo;
  String timestamp = "Hora desconocida";
  if (getLocalTime(&timeinfo)) {
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%d/%m/%Y %H:%M:%S", &timeinfo);
    timestamp = String(timeStringBuff);
  }

  // Unimos el mensaje personalizado con la marca de tiempo
  String caption = mensajeTelegram + "\n🕒 " + timestamp;

  enviarFotoTelegram(CHAT_ID_PERMITIDO, fb->buf, fb->len, caption);
  
  // Liberar la PSRAM (Paso crítico)
  esp_camera_fb_return(fb);
}

void onEnterInit() {
    Serial.println("Entrando en STATE_INITIALIZING");
    servos_init();
    cargarHorarios();
    cargarCalibraciones();
    cargarVelocidades();

    if (!initCamera()) {
        Serial.println("Sistema detenido por fallo de hardware.");
        while(true) { vTaskDelay(100); } // Bloquear si no hay cámara
    }

    // 1. Conexión WiFi inicial
    Serial.print("Conectando a WiFi ");
    WiFi.begin(currentSSID.c_str(), currentPASS.c_str());

    // Esperar conexión (bloqueante)
    // IMPORTANTE: Si se va la luz y vuelve pero no hay internet, 
    // el robot se quedará aquí "congelado" y no atacará palomas.
    // ¿Es esto lo que quieres? Si sí, está perfecto.
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Opcional: Si tarda mucho, podrías reiniciar el ESP
    retryCount++;
    if(retryCount > 60) { // 30 segundos
        Serial.println("\nError WiFi: Reiniciando...");
        ESP.restart();
    }
    }

    Serial.println("\nWiFi Conectado.");

    // Configurar el servidor NTP (UTC-4 son -14400 segundos de desfase)
    configTime(-14400, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("Hora sincronizada por NTP.");

    // Configurar SSL
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 

    // --- SEÑAL VERDE ---
    // Avisamos a TaskTelegram que ya puede empezar a trabajar
    wifiSystemReady = true;
    FSMEvent e = EVENT_INIT_COMPLETE;
    xQueueSend(fsmQueue, &e, 0);
}

void onEnterIdle() {
    Serial.println("Entrando en STATE_IDLE");
    Laser_01.off();
    stopPattern(g_homeX[AREA_ACTUAL], g_homeY[AREA_ACTUAL]);
    xTimerStop(monitoringTimer, 0);
    xTimerStart(updateIdleLogicTimer, 0);
}

void onEnterMonitoring(){
    Serial.println("Entrando en STATE_MONITORING");
    Laser_01.off();
    xTimerStart(monitoringTimer, 0);
}

void onEnterPicturePreAttack() {
    Serial.println("Entrando en STATE_PICTURE_PRE_ATTACK");
    xTimerStop(monitoringTimer, 0); // Detenemos el monitoreo

    // Llamamos a la función auxiliar con el texto de intrusión
    capturarYEnviarFoto("🐦 ¡Posible paloma detectada! Iniciando secuencia láser de disuasión.");

    // Disparar el evento para pasar a STATE_ATTACKING
    FSMEvent e = EVENT_PROCESSING_COMPLETE;
    xQueueSend(fsmQueue, &e, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void onEnterPicturePostAttack() {
    Serial.println("Entrando en STATE_PICTURE_POST_ATTACK");

    // Llamamos a la función auxiliar con el texto de verificación
    capturarYEnviarFoto("✅ Secuencia láser finalizada. Verificando si el objetivo abandonó el área.");

    // Disparar el evento para volver a STATE_MONITORING o STATE_IDLE
    FSMEvent e = EVENT_PROCESSING_COMPLETE;
    xQueueSend(fsmQueue, &e, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void onEnterAttacking(){
    Serial.println("Entrando en STATE_ATTACKING");
    xTimerStop(monitoringTimer, 0);

    // Lógica para elegir patrón aleatorio o rotativo
    PatternType p = (random() % 2 == 0) ? PATTERN_ZIGZAG_HORIZ : PATTERN_ZIGZAG_VERT;
    Laser_01.on();

    // ✅ 1. Definimos cuántas áreas vamos a atacar en este ciclo
    patCtx.totalAreas = 4; // Asegúrate de agregar esta variable a tu struct
    patCtx.areaPhase = 0;  // Empezamos por la primera (fase 0)

    patCtx.targetArea[0] = AREA_ACTUAL;

    int index = 1;
    for (int i = 0; i < patCtx.totalAreas; i++) {
        if (i != AREA_ACTUAL) {
            patCtx.targetArea[index] = i;
            index++;
        }
    }
    startPattern(p,
        g_calibMinX[patCtx.targetArea[0]],
        g_calibMaxX[patCtx.targetArea[0]],
        g_calibMinY[patCtx.targetArea[0]],
        g_calibMaxY[patCtx.targetArea[0]]
    );
}

void onEnterCalibSetLL(){
    Serial.println("Entrando en STATE_CALIB_SET_LL");
    Laser_01.on();
}

void onEnterCalibSetUR(){
    Serial.println("Entrando en STATE_CALIB_SET_UR");
    temp_X1 = SERVO_X.getPosition();
    temp_Y1 = SERVO_Y.getPosition();
        
    Serial.println("Punto 1 capturado. Mueve al Punto 2.");
}

void onEnterCalibSave(){
    temp_X2 = SERVO_X.getPosition();
    temp_Y2 = SERVO_Y.getPosition();
    Serial.println("Punto 2 capturado.");


    // Guardamos en el índice del área actual
    g_calibMinX[AREA_ACTUAL] = min(temp_X1, temp_X2);
    g_calibMaxX[AREA_ACTUAL] = max(temp_X1, temp_X2);
    g_calibMinY[AREA_ACTUAL] = min(temp_Y1, temp_Y2);
    g_calibMaxY[AREA_ACTUAL] = max(temp_Y1, temp_Y2);

    g_homeX[AREA_ACTUAL] = g_calibMinX[AREA_ACTUAL] + ((g_calibMaxX[AREA_ACTUAL] - g_calibMinX[AREA_ACTUAL]) / 2);
    g_homeY[AREA_ACTUAL] = g_calibMinY[AREA_ACTUAL] + ((g_calibMaxY[AREA_ACTUAL] - g_calibMinY[AREA_ACTUAL]) / 2);

    // Llamamos a la nueva función de guardado
    guardarCalibracion(AREA_ACTUAL);

    Serial.printf("Calibracion Área %d: X[%i - %i], Y[%i - %i]\n", 
                AREA_ACTUAL + 1, g_calibMinX[AREA_ACTUAL], g_calibMaxX[AREA_ACTUAL], 
                g_calibMinY[AREA_ACTUAL], g_calibMaxY[AREA_ACTUAL]);
    vTaskDelay(pdMS_TO_TICKS(3000));
    FSMEvent e = EVENT_CALIBRATION_DONE;
    xQueueSend(fsmQueue, &e, 0);
}

void onEnterCalibPrev(){
    Serial.println("Entrando en STATE_CALIB_PREVIEW");
    Laser_01.on();
    startPattern(PATTERN_RECTANGLE_PREVIEW, g_calibMinX[AREA_ACTUAL], g_calibMaxX[AREA_ACTUAL], g_calibMinY[AREA_ACTUAL], g_calibMaxY[AREA_ACTUAL]);
}

// FSM principal
void TaskFSM(void *pvParameters) {
    (void) pvParameters;

    FSMEvent receivedEvent;
    SystemState currentState = STATE_INITIALIZING; // Estado inicial explícito
    SystemState newState = STATE_INITIALIZING; // Estado inicial explícito

    // Crear el timer (5 minutos, one-shot o auto-reload según prefieras logicamente)
    // Aquí uso auto-reload false para controlarlo manualmente en los estados
    monitoringTimer = xTimerCreate("MonTimer", pdMS_TO_TICKS(MONITORING_INTERVAL_MS), pdFALSE, 0, monitoringTimerCallback);
    updateIdleLogicTimer = xTimerCreate("IdleLogicTimer", pdMS_TO_TICKS(60000), pdTRUE, 0, updateIdleLogicTimerCallback); // Chequeo cada minuto

    // Establecer estado inicial y ejecutar su acción de entrada
    setState(currentState);
    onEnterInit();

    for (;;) {
    if (xQueueReceive(fsmQueue, &receivedEvent, portMAX_DELAY)) {
        SystemState state = getState();
        
        newState = currentState; // Por defecto, no hay cambio de estado
        
        // LÓGICA DE TRANSICIÓN: solo decide el siguiente estado
        switch (currentState) {
        case STATE_INITIALIZING:
            if (receivedEvent == EVENT_INIT_COMPLETE) newState = STATE_IDLE;
            break;
        
        case STATE_IDLE:
            if (receivedEvent == EVENT_START_COMMAND) newState = STATE_MONITORING;
            else if (receivedEvent == EVENT_ENTER_CALIBRATION) newState = STATE_CALIB_SET_LL;
            else if (receivedEvent == EVENT_TAKE_PICTURE) newState = STATE_PICTURE_POST_ATTACK;
            else if (receivedEvent == EVENT_ENTER_PREVIEW) newState = STATE_CALIB_PREVIEW;
            break;

        case STATE_MONITORING:
            if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            else if (receivedEvent == EVENT_PIGEON_DETECTED || receivedEvent == EVENT_MANUAL_COMMAND ) newState = STATE_PICTURE_PRE_ATTACK;
            else if (receivedEvent == EVENT_NO_PIGEON || receivedEvent == EVENT_TIMER_EXPIRED) newState = STATE_PICTURE_PRE_ATTACK;
            else if (receivedEvent == EVENT_TAKE_PICTURE) newState = STATE_PICTURE_POST_ATTACK;
            break;

        case STATE_PICTURE_PRE_ATTACK:
            if (receivedEvent == EVENT_PROCESSING_COMPLETE) newState = STATE_ATTACKING;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;

        case STATE_ATTACKING:
            if (receivedEvent == EVENT_ATTACK_COMPLETE) newState = STATE_PICTURE_POST_ATTACK;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;

        case STATE_PICTURE_POST_ATTACK:
            if (receivedEvent == EVENT_PROCESSING_COMPLETE) newState = STATE_MONITORING;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;

        case STATE_CALIB_SET_LL:
            if (receivedEvent == EVENT_CONFIRM_POINT) newState = STATE_CALIB_SET_UR;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;

        case STATE_CALIB_SET_UR:
            if (receivedEvent == EVENT_CONFIRM_POINT) newState = STATE_CALIB_SAVE;
            break;

        case STATE_CALIB_SAVE:
            if (receivedEvent == EVENT_CALIBRATION_DONE) newState = STATE_CALIB_PREVIEW;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;
        case STATE_CALIB_PREVIEW:
            if (receivedEvent == EVENT_PREVIEW_DONE) newState = STATE_IDLE;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;
        case STATE_ERROR:
            if (receivedEvent == EVENT_RESUME) newState = STATE_IDLE;
            break;
        }

        // APLICAR CAMBIO DE ESTADO Y EJECUTAR ACCIÓN DE ENTRADA
        if (newState != currentState) {
            setState(newState);

            switch (newState) {
                case STATE_INITIALIZING: onEnterInit(); break;
                case STATE_IDLE: onEnterIdle(); break;
                case STATE_MONITORING: onEnterMonitoring(); break;
                case STATE_PICTURE_PRE_ATTACK: onEnterPicturePreAttack(); break;
                case STATE_PICTURE_POST_ATTACK: onEnterPicturePostAttack(); break;
                case STATE_ATTACKING: onEnterAttacking(); break;
                case STATE_CALIB_SET_LL: onEnterCalibSetLL(); break;
                case STATE_CALIB_SET_UR: onEnterCalibSetUR(); break;
                case STATE_CALIB_SAVE: onEnterCalibSave(); break;
                case STATE_CALIB_PREVIEW: onEnterCalibPrev(); break;
            }
        }
        currentState = newState; // Actualizar estado actual
    }
    }
}

void TaskServoControl(void *pvParameters) {
    (void) pvParameters;

    // Aseguramos que no haya patrón activo al inicio
    patCtx.active = false;

    vTaskDelay(pdMS_TO_TICKS(1000));
    ManualPosCmd receivedCmd;

    for (;;) {
    // A. ¿Hay un patrón automático activo?

    if (patCtx.active) {
        updatePatternLogic();
        xQueueReset(manualControlQueue); 
    } 
    // B. Si NO hay patrón, verificamos si hay comandos manuales
    else {
        // Revisamos la cola.
        if (xQueueReceive(manualControlQueue, &receivedCmd, 0) == pdTRUE) {
            
            if (receivedCmd.cmdType == 0) {
                // Comando para X
                SERVO_X.setTarget((int)receivedCmd.value);
                
            } else {
                // Comando para Y
                SERVO_Y.setTarget((int)receivedCmd.value);
            }
        }
    }

    // C. Mover los servos físicamente hacia el objetivo actual
    SERVO_X.update();
    SERVO_Y.update();

    vTaskDelay(pdMS_TO_TICKS(SPEED_MS)); 
    }
}

void TaskComms(void *pvParameters) {
    (void) pvParameters;

    TickType_t lastHeartbeat = xTaskGetTickCount();

    for (;;) {
    // Comunicación Serial con Mini PC

    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        
        if (msg.equals("CMD,START")) {
        FSMEvent e = EVENT_START_COMMAND;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,NAVIGATE");
        } 
        else if (msg.equals("CMD,STOP")) {
        FSMEvent e = EVENT_STOP_COMMAND;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,STOP");
        }
        else if (msg.equals("CMD,RESUME")) {
        FSMEvent e = EVENT_RESUME;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,RESUME");
        }
        else if (msg.equals("X:")) {
            // Extraer el valor después de "X:"
            String valStr = msg.substring(2); // 2 es la longitud de "X:"
            ManualPosCmd cmd;
            cmd.cmdType = 0; // 0 para Eje X
            cmd.value = valStr.toInt();

            // Enviar a la cola del Servo (no bloqueante)
            xQueueSend(manualControlQueue, &cmd, 0);

            Serial.println(cmd.value);
        }
        else if (msg.equals("Y:")) {
            // Extraer el valor después de "Y:"
            String valStr = msg.substring(2); // 2 es la longitud de "Y:"
            ManualPosCmd cmd;
            cmd.cmdType = 1; // 1 para Eje Y
            cmd.value = valStr.toInt();

            // Enviar a la cola del Servo (no bloqueante)
            xQueueSend(manualControlQueue, &cmd, 0);

            Serial.println(cmd.value);
        }
        else if (msg.equals("CONFIRM_LL")) {
            // Extraer el valor después de "Y:"
            Serial.println("DEBUG: Comando CONFIRM_LL recibido");

            FSMEvent e = EVENT_CONFIRM_POINT;
            xQueueSend(fsmQueue, &e, 0);;
        }
        else if (msg.equals("CONFIRM_UR")) {
            // Extraer el valor después de "Y:"
            Serial.println("DEBUG: Comando CONFIRM_UR recibido");

            FSMEvent e = EVENT_CONFIRM_POINT;
            xQueueSend(fsmQueue, &e, 0);;
        }
        else if (msg.equals("SimulatePigeon")) {
            // Extraer el valor después de "Y:"
            Serial.println("DEBUG: Comando SimulatePigeon recibido");

            FSMEvent e = EVENT_PIGEON_DETECTED;
            xQueueSend(fsmQueue, &e, 0);;
        }

        else {
        Serial.println("ERROR,UNKNOWN_CMD");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskTelegram(void *pvParameters) {
    (void) pvParameters;

    // --- ESPERA DE SEGURIDAD ---
    // No hacer NADA hasta que onEnterInit termine la conexión inicial
    while (!wifiSystemReady) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    bool firstRun = true;
    Serial.println("Telegram Task: Iniciando polling...");
    for (;;) {
    // Verificar conexión WiFi y reconectar si es necesario
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi perdido, reconectando...");
        WiFi.disconnect();
        WiFi.reconnect();
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
    }
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    if (getState() == STATE_IDLE && firstRun) {
        enviarMenu(CHAT_ID_PERMITIDO);
        firstRun = false;
    }
    vTaskDelay(pdMS_TO_TICKS(BOT_MTBS)); 
    }
}