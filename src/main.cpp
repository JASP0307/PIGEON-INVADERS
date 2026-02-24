#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include "ServoModule.h"
#include "IO_Points.h"
#include "Laser.h"
#include "SystemDefinitions.h"

#define btSerial Serial1

#define ZIGZAG_STEP_SIZE 10.0f // Cuánto avanza en el eje secundario
#define TARGET_TOLERANCE 2.0f  // Margen de error para decir "llegué"

// Credenciales Wi-Fi
#define WIFI_SSID "FamiliaSuárez"
#define WIFI_PASSWORD "RigobertoSuarez"

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
QueueHandle_t telemetryQueue;

SemaphoreHandle_t stateMutex;

ServoModule SERVO_X(Pinout::ServoMotors::SERVO_X);
ServoModule SERVO_Y(Pinout::ServoMotors::SERVO_Y);

Laser Laser_01(Pinout::Laser::Laser_1);

// Tiempos de simulación en milisegundos
TickType_t monitoringStartTime  = 0;
// Definir un Handle para el timer de FreeRTOS
TimerHandle_t monitoringTimer;

TaskHandle_t xHandleServos;

// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskTelegram(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);

// Funciones auxiliares del brazo delta
void servos_init();

// Funciones auxiliares
SystemState getState();
void setState(SystemState newState);

void setup() {
  Serial.begin(115200);
  btSerial.begin(9600); 
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
  telemetryQueue = xQueueCreate(5, sizeof(SystemState));
  if (telemetryQueue == NULL) {
    Serial.println("Error: No se pudo crear la cola telemetryQueue.");
    while (1);
  }

  // Crear cola FSM
  manualControlQueue = xQueueCreate(5, sizeof(ManualPosCmd));
  if (manualControlQueue == NULL) {
    Serial.println("Error: No se pudo crear la cola manualControlQueue.");
    while (1);
  }

  pinMode(Pinout::TiraLED::LEDs, OUTPUT);
 
  // Crear tareas
  xTaskCreatePinnedToCore(TaskTelegram,               "Telegram",     6144, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskComms,                  "Comms",        2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskFSM,                    "FSM",          4096, NULL, 3, NULL, 1);
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

// Función para iniciar un patrón (llamar desde la FSM Principal)
void startPattern(PatternType type, float min_x, float max_x, float min_y, float max_y) {
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
                        FSMEvent e = EVENT_CALIBRATION_DONE;
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
                     // Terminamos el barrido, volver a home
                    SERVO_X.setTarget(patCtx.minX);
                    SERVO_Y.setTarget(patCtx.minY);
                    patCtx.active = false;
                    
                    FSMEvent e = EVENT_ATTACK_COMPLETE;
                    xQueueSend(fsmQueue, &e, 0);
                } else {
                    // Si step es par, vamos a la derecha (MaxX), si es impar a la izquierda (MinX)
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
                    FSMEvent e = EVENT_ATTACK_COMPLETE;
                    xQueueSend(fsmQueue, &e, 0);
                } else {
                    float targetY = (patCtx.stepIndex % 2 == 0) ? patCtx.maxY : patCtx.minY;
                    SERVO_X.setTarget(nextX);
                    SERVO_Y.setTarget(targetY);
                    patCtx.stepIndex++;
                }
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

void enviarTecladoCalibracion(String chat_id) {
  // Construimos el teclado en formato JSON
  String keyboardJson = "[";
  // Fila 1: Arriba
  keyboardJson += "[{\"text\":\"⬆️ Subir\", \"callback_data\":\"TILT_UP\"}],";
  // Fila 2: Izquierda, Guardar, Derecha
  keyboardJson += "[{\"text\":\"⬅️ Izq\", \"callback_data\":\"PAN_LEFT\"},";
  keyboardJson += "{\"text\":\"💾 Guardar Límite\", \"callback_data\":\"SAVE_LIMIT\"},";
  keyboardJson += "{\"text\":\"Der ➡️\", \"callback_data\":\"PAN_RIGHT\"}],";
  // Fila 3: Abajo
  keyboardJson += "[{\"text\":\"⬇️ Bajar\", \"callback_data\":\"TILT_DOWN\"}]";
  keyboardJson += "]";

  bot.sendMessageWithInlineKeyboard(chat_id, "Modo Calibración Activado. Usa las flechas para mover el láser:", "", keyboardJson);
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
    else if (text == "TILT_UP") {
        currentTilt += stepSize;
        cmd.cmdType = 1;   // Eje Y
        cmd.value = currentTilt;
    }
    else if (text == "PAN_LEFT") {
        currentPan -= stepSize;
        cmd.cmdType = 0;   // Eje X
        cmd.value = currentPan;
    }
    else if (text == "PAN_RIGHT") {
        currentPan += stepSize;
        cmd.cmdType = 0;   // Eje X
        cmd.value = currentPan;
    }
    else if (text == "SAVE_LIMIT") {
        FSMEvent e = EVENT_CONFIRM_POINT;
        xQueueSend(fsmQueue, &e, 0);

        bot.sendMessage(chat_id,
            "Límite guardado en:\nPan: " + String(currentPan) +
            "°\nTilt: " + String(currentTilt) + "°",
            "");

        return;  // No continuar
    }
    else {
        movimientoValido = false;
    }

    if (!movimientoValido) return;

    // Enviar a la cola sin bloquear
    if (xQueueSend(manualControlQueue, &cmd, 0) == pdPASS) {
        bot.sendMessage(chat_id,
            "Posición actual:\nPan: " + String(currentPan) +
            "°\nTilt: " + String(currentTilt) + "°",
            "");
    }
    else {
        bot.sendMessage(chat_id, "⚠ Cola ocupada. Intente nuevamente.", "");
    }
}

void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    String text = bot.messages[i].text;
  

    // Seguridad: Ignorar mensajes de extraños
    if (chat_id != CHAT_ID_PERMITIDO) {
        bot.sendMessage(chat_id, "Acceso denegado.", "");
        continue;
    }

    if (bot.messages[i].type == "callback_query") {
      bot.answerCallbackQuery(bot.messages[i].query_id);
    }

    if ((getState() == STATE_CALIB_SET_LL || 
        getState() == STATE_CALIB_SET_UR) && 
        bot.messages[i].type == "callback_query") {

      procesarMovimientoManual(text, chat_id);
    }

    if (text == "/start") {
      FSMEvent e = EVENT_START_COMMAND;
      xQueueSend(fsmQueue, &e, 0);
      bot.sendMessage(chat_id, "Sistema ARMADO y en MONITORING.", "Markdown");
    }
    else if (text == "/stop") {
      FSMEvent e = EVENT_STOP_COMMAND;
      xQueueSend(fsmQueue, &e, 0);
      bot.sendMessage(chat_id, "Sistema DETENIDO.", "");
    }
    else if (text == "/status") {
      // Pedimos el estado actual de forma thread-safe
      SystemState currentState = getState(); 
      String stateStr = SystemStateToString(currentState);
      bot.sendMessage(chat_id, "Estado actual: " + stateStr, "");
    }
    else if (text == "/pigeonsim") {
      FSMEvent e = EVENT_PIGEON_DETECTED;
      xQueueSend(fsmQueue, &e, 0);
      bot.sendMessage(chat_id, "¡Paloma Detectada!", "");
    }
    else if (text == "/calibrar") {
      FSMEvent e = EVENT_ENTER_CALIBRATION;
      xQueueSend(fsmQueue, &e, 0);
      enviarTecladoCalibracion(chat_id);
    }
    else if (text == "/help") {
      String welcome = "Comandos Pigeon Invaders:\n";
      welcome += "/start : Iniciar monitoreo\n";
      welcome += "/stop : Detener sistema\n";
      welcome += "/status : Ver estado actual\n";
      welcome += "/pigeonsim : Simular detección de paloma\n";
      bot.sendMessage(chat_id, welcome, "");
    }
  }
}

void onEnterInit() {
  Serial.println("Entrando en STATE_INITIALIZING");
  //servos_init();

  // 1. Conexión WiFi inicial
  Serial.print("Conectando a WiFi ");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
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

  // Configurar SSL
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 

  // --- SEÑAL VERDE ---
  // Avisamos a TaskTelegram que ya puede empezar a trabajar
  wifiSystemReady = true;
  Laser_01.on();
  FSMEvent e = EVENT_INIT_COMPLETE;
  xQueueSend(fsmQueue, &e, 0);
}

void onEnterIdle() {
  Serial.println("Entrando en STATE_IDLE");
  //vTaskDelay(pdMS_TO_TICKS(1000));
  Laser_01.off();
  digitalWrite(Pinout::TiraLED::LEDs, LOW);
  xTimerStop(monitoringTimer, 0);
}

void onEnterMonitoring(){
  Serial.println("Entrando en STATE_MONITORING");
  Laser_01.off();
  xTimerStart(monitoringTimer, 0);
}
  
void onEnterAttacking(){
  Serial.println("Entrando en STATE_ATTACKING");
  xTimerStop(monitoringTimer, 0);
  
  // Lógica para elegir patrón aleatorio o rotativo
  PatternType p = (random() % 2 == 0) ? PATTERN_ZIGZAG_HORIZ : PATTERN_ZIGZAG_VERT;
  Laser_01.on();
  startPattern(p, g_calibMinX, g_calibMaxX, g_calibMinY, g_calibMaxY);
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

void onEnterCalibPrev(){
  Serial.println("Entrando en STATE_CALIB_PREVIEW");
  temp_X2 = SERVO_X.getPosition();
  temp_Y2 = SERVO_Y.getPosition();
  Serial.println("Punto 2 capturado.");
  g_calibMinX = min(temp_X1, temp_X2);
  g_calibMaxX = max(temp_X1, temp_X2);
  
  g_calibMinY = min(temp_Y1, temp_Y2);
  g_calibMaxY = max(temp_Y1, temp_Y2);

  Serial.printf("Calibracion Final: X[%f - %f], Y[%f - %f]\n", 
                g_calibMinX, g_calibMaxX, g_calibMinY, g_calibMaxY);
  vTaskDelay(pdMS_TO_TICKS(3000));  
  startPattern(PATTERN_RECTANGLE_PREVIEW, g_calibMinX, g_calibMaxX, g_calibMinY, g_calibMaxY);
}

// FSM principal
void TaskFSM(void *pvParameters) {
  (void) pvParameters;

  FSMEvent receivedEvent;
  SystemState currentState = STATE_INITIALIZING; // Estado inicial explícito
  SystemState newState = STATE_INITIALIZING; // Estado inicial explícito

  // Crear el timer (5 minutos, one-shot o auto-reload según prefieras logicamente)
  // Aquí uso auto-reload false para controlarlo manualmente en los estados
  monitoringTimer = xTimerCreate("MonTimer", pdMS_TO_TICKS(3000), pdFALSE, 0, monitoringTimerCallback);
  
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
            break;

        case STATE_MONITORING:
            if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            else if (receivedEvent == EVENT_PIGEON_DETECTED || receivedEvent == EVENT_MANUAL_COMMAND) newState = STATE_ATTACKING;
            else if (receivedEvent == EVENT_NO_PIGEON || receivedEvent == EVENT_TIMER_EXPIRED) {
              newState = STATE_MONITORING; // Reiniciar monitoreo (puede ser útil para resetear el timer o alguna variable de conteo)
              onEnterMonitoring(); // Reiniciar monitoreo para el próximo ciclo
            }
            break;
            
        case STATE_ATTACKING:
            if (receivedEvent == EVENT_ATTACK_COMPLETE) newState = STATE_MONITORING;
            break;
        
        case STATE_CALIB_SET_LL:
            if (receivedEvent == EVENT_CONFIRM_POINT) newState = STATE_CALIB_SET_UR;
            else if (receivedEvent == EVENT_STOP_COMMAND) newState = STATE_IDLE;
            break;

        case STATE_CALIB_SET_UR:
            if (receivedEvent == EVENT_CONFIRM_POINT) newState = STATE_CALIB_PREVIEW;
            break;
        
        case STATE_CALIB_PREVIEW:
            if (receivedEvent == EVENT_CALIBRATION_DONE) newState = STATE_IDLE;
            break;
        case STATE_ERROR:
            if (receivedEvent == EVENT_RESUME) newState = STATE_IDLE;
            break;
      }

      // APLICAR CAMBIO DE ESTADO Y EJECUTAR ACCIÓN DE ENTRADA
      if (newState != currentState) {
          setState(newState);
          xQueueSend(telemetryQueue, &currentState, 0);

          switch (newState) {
              case STATE_INITIALIZING: onEnterInit(); break;
              case STATE_IDLE: onEnterIdle(); break;
              case STATE_MONITORING: onEnterMonitoring(); break;
              case STATE_ATTACKING: onEnterAttacking(); break;
              case STATE_CALIB_SET_LL: onEnterCalibSetLL(); break;
              case STATE_CALIB_SET_UR: onEnterCalibSetUR(); break;
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
    
    vTaskDelay(pdMS_TO_TICKS(50)); 
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
    
    // Enviar heartbeat periódico usando FreeRTOS ticks
    if ((xTaskGetTickCount() - lastHeartbeat) > pdMS_TO_TICKS(5000)) {
      SystemState state = getState();
      //Serial.print("HEARTBEAT,");
      //Serial.print(state);
      //Serial.println();
      lastHeartbeat = xTaskGetTickCount();
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

    vTaskDelay(pdMS_TO_TICKS(BOT_MTBS)); 
  }
}