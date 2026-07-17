#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

#include "IO_Points.h"
#include "SystemDefinitions.h"
#include "credentials.h"
#include "wifi_manager.h"
#include "camera_manager.h"
#include "telegram_bot.h"
#include "servo_patterns.h"
#include "calibration.h"
#include "schedule.h"

// =================================================================
// DEFINICIONES DE CREDENCIALES
// (Declaradas como extern en credentials.h)
// =================================================================

const char* SSID_FAMILIA = "FamiliaSuárez";
const char* PASS_FAMILIA = "RigobertoSuarez";
const char* SSID_CLARO = "CLAROV6RCY";
const char* PASS_CLARO = "48575443AFC7839E";
const char* BOT_TOKEN = "8378042870:AAG3z-YlLjb98I--cp8aATP29ybx_0LAfKg";
const char* CHAT_ID_PERMITIDO = "8210739066";

// =================================================================

// =================================================================
// DEFINICIONES DE VARIABLES GLOBALES
// (Declaradas como extern en SystemDefinitions.h)
// =================================================================

volatile SystemState currentState = STATE_INITIALIZING;

int AREA_ACTUAL = 0;

PatternContext patCtx;

int stepSize = 2;
int stepSizefast = 10;

int MONITORING_INTERVAL_MS = (2 * 60 * 1000);

int g_calibMinX[AREAS] = {0, 0, 0, 0};
int g_calibMaxX[AREAS] = {100, 100, 100, 100};
int g_calibMinY[AREAS] = {0, 0, 0, 0};
int g_calibMaxY[AREAS] = {100, 100, 100, 100};

int g_homeX[AREAS] = {90, 90, 90, 90};
int g_homeY[AREAS] = {90, 90, 90, 90};

int currentPan = 90;
int currentTilt = 90;

RangoHorario horariosMonitor[NUM_HORARIOS];

int temp_X1 = 0, temp_Y1 = 90;
int temp_X2 = 90, temp_Y2 = 0;

volatile bool wifiSystemReady = false;

Preferences preferences;

int SPEED_MS = 50;
const int HOME_SPEED_MS = 50;
bool initialHomeMovement = false;

// =================================================================

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

// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskServoControl(void *pvParameters);

// Funciones auxiliares
void servos_init();
void monitoringTimerCallback(TimerHandle_t xTimer);
void updateIdleLogicTimerCallback(TimerHandle_t xTimer);

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
    xTaskCreatePinnedToCore(telegram_task,               "Telegram",     10240, NULL, 1, NULL, 0);
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
    int savedX = cargarPosicionServosX();
    int savedY = cargarPosicionServosY();
    SERVO_X.begin(savedX);
    SERVO_Y.begin(savedY);
    Serial.printf("Servos inicializados en posición guardada: X=%d, Y=%d\n", savedX, savedY);
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

void capturarYEnviarFoto(String mensajeTelegram) {
  camera_fb_t * fb = camera_capture();
  
  if (!fb) {
    telegram_sendMessage(CHAT_ID_PERMITIDO, "Fallo en la camara al intentar capturar el area.");
    return;
  } 
  
  struct tm timeinfo;
  String timestamp = "Hora desconocida";
  if (getLocalTime(&timeinfo)) {
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%d/%m/%Y %H:%M:%S", &timeinfo);
    timestamp = String(timeStringBuff);
  }

  String caption = mensajeTelegram + "\n" + timestamp;

  camera_sendToTelegram(CHAT_ID_PERMITIDO, BOT_TOKEN, fb->buf, fb->len, caption);
  
  camera_releaseFrame(fb);
}

void onEnterInit() {
    Serial.println("Entrando en STATE_INITIALIZING");
    servos_init();
    cargarHorarios();
    cargarCalibraciones();
    cargarVelocidades();

    if (!camera_init()) {
        Serial.println("Sistema detenido por fallo de hardware.");
        while(true) { vTaskDelay(100); } // Bloquear si no hay cámara
    }

    // Conexión WiFi inicial (bloqueante)
    wifi_connect();

    // Configurar el servidor NTP (UTC-4 son -14400 segundos de desfase)
    configTime(-14400, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("Hora sincronizada por NTP.");

    // Inicializar el bot de Telegram
    telegram_init(BOT_TOKEN, CHAT_ID_PERMITIDO);

    // --- SEÑAL VERDE ---
    // Avisamos a TaskTelegram que ya puede empezar a trabajar
    wifiSystemReady = true;
    FSMEvent e = EVENT_INIT_COMPLETE;
    xQueueSend(fsmQueue, &e, 0);
}

void onEnterIdle() {
    Serial.println("Entrando en STATE_IDLE");
    Laser_01.off();
    initialHomeMovement = true;
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
    int idleCounter = 0;

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

    if (initialHomeMovement) {
        if (SERVO_X.getPosition() == SERVO_X.getTarget() &&
            SERVO_Y.getPosition() == SERVO_Y.getTarget()) {
            initialHomeMovement = false;
            guardarPosicionServos(SERVO_X.getPosition(), SERVO_Y.getPosition());
            Serial.println("Home movement complete, resuming normal speed");
        }
        vTaskDelay(pdMS_TO_TICKS(HOME_SPEED_MS));
    } else {
        // Guardar posición periódicamente cuando los servos están en reposo
        if (SERVO_X.getPosition() == SERVO_X.getTarget() &&
            SERVO_Y.getPosition() == SERVO_Y.getTarget()) {
            idleCounter++;
            if (idleCounter >= 600) { // ~30s a 50ms
                idleCounter = 0;
                guardarPosicionServos(SERVO_X.getPosition(), SERVO_Y.getPosition());
            }
        } else {
            idleCounter = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(SPEED_MS)); 
    }
    }
}