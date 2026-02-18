// Máquina de Estados para Arduino Mega - Robot de Desmalezado

#include <Arduino.h>
#include <queue.h>
#include <semphr.h>
#include "ServoModule.h"
#include "IO_Points.h"
#include "Laser.h"

#define btSerial Serial1

#define ZIGZAG_STEP_SIZE 10.0f // Cuánto avanza en el eje secundario
#define TARGET_TOLERANCE 2.0f  // Margen de error para decir "llegué"

// Handles globales
QueueHandle_t fsmQueue;
QueueHandle_t servoQueue;
SemaphoreHandle_t stateMutex;

ServoModule SERVO_X(Pinout::BrazoDelta::SERVO_X);
ServoModule SERVO_Y(Pinout::BrazoDelta::SERVO_Y);

Laser Laser_01(Pinout::Laser::Laser_1);

// Tiempos de simulación en milisegundos
TickType_t monitoringStartTime  = 0;
// Definir un Handle para el timer de FreeRTOS
TimerHandle_t monitoringTimer;

TaskHandle_t xHandleServos;

// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);
void TaskBluetoothCommunication(void *pvParameters);

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

  servoQueue = xQueueCreate(5, sizeof(int));

  pinMode(Pinout::TiraLED::LEDs, OUTPUT);
  
  Serial.println("Motores inicializados");
 
  // Crear tareas
  xTaskCreate(TaskFSM,                    "FSM",        512, NULL, 3, NULL);
  xTaskCreate(TaskSensors,                "Sensors",    256, NULL, 2, NULL);
  xTaskCreate(TaskComms,                  "Comms", 256, NULL, 2, NULL);
  xTaskCreate(TaskServoControl,           "ServoControl", 256, NULL, 3, &xHandleServos);
  xTaskCreate(TaskBluetoothCommunication, "Bluetooth Test Task", 1024, NULL, 2, NULL);
  
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
    //Serial.println("Brazo Delta inicializado y en HOME.");
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
                float nextY = patCtx.minY + (patCtx.stepIndex * ZIGZAG_STEP_SIZE);
                
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
                float nextX = patCtx.minX + (patCtx.stepIndex * ZIGZAG_STEP_SIZE);
                
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


void onEnterInit() {
  servos_init();
}

void onEnterIdle() {
  //Serial.println("Entrando en IDLE");
  //vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(Pinout::TiraLED::LEDs, LOW);
  xTimerStop(monitoringTimer, 0);
}

void onEnterMonitoring(){
  xTimerStart(monitoringTimer, 0);
}
  
void onEnterAttacking(){
  xTimerStop(monitoringTimer, 0);
  
  // Lógica para elegir patrón aleatorio o rotativo
  startPattern(PATTERN_RECTANGLE_PREVIEW, g_calibMinX, g_calibMaxX, g_calibMinY, g_calibMaxY);
  Laser_01.on();
  startPattern(p, g_calibMinX, g_calibMaxX, g_calibMinY, g_calibMaxY);
}
void onEnterCalibSetLL(){
  
}
void onEnterCalibSetUR(){
  
}

void onEnterCalibPrev(){
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
  monitoringTimer = xTimerCreate("MonTimer", pdMS_TO_TICKS(300000), pdFALSE, 0, monitoringTimerCallback);
  
  // Establecer estado inicial y ejecutar su acción de entrada
  setState(currentState);
  onEnterInit();

  for (;;) {
    if (xQueueReceive(fsmQueue, &receivedEvent, portMAX_DELAY))) {
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
            else if (receivedEvent == EVENT_TIMER_EXPIRED || receivedEvent == EVENT_NO_PIGEON){
              newState = STATE_MONITORING;
              onEnterMonitoring();
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
            // El evento de fin de espera ahora viene de la cola
            if (receivedEvent == EVENT_CALIBRATION_DONE) newState = STATE_IDLE;
            break;
        case ERROR_STATE:
            if (receivedEvent == EVENT_RESUME) newState = STATE_IDLE;
            break;
      }
    }

      // APLICAR CAMBIO DE ESTADO Y EJECUTAR ACCIÓN DE ENTRADA
      if (newState != currentState) {
          setState(newState);

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
    }

    // GESTIÓN DE TIMEOUTS
    currentState = getState();
    if (currentState == STATE_MONITORING) {
        if ((xTaskGetTickCount() - monitoringStartTime) >= pdMS_TO_TICKS(300000)) {
            FSMEvent e = EVENT_TIMER_EXPIRED;
            xQueueSend(fsmQueue, &e, 0);
        }
    }  
  }
}

void TaskServoControl(void *pvParameters) {
  (void) pvParameters;
  
  // Aseguramos que no haya patrón activo al inicio
  patCtx.active = false;
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  for (;;) {
    // 1. Calcular nuevos objetivos si estamos en modo automático (patrón)
    updatePatternLogic();
    
    // 2. Mover los servos físicamente hacia el objetivo actual
    SERVO_X.update();
    SERVO_Y.update();
    vTaskDelay(pdMS_TO_TICKS(75)); 
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
      
      if (msg.equals("CMD,NAVIGATE")) {
        FSMEvent e = EVENT_NAVIGATE;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,NAVIGATE");
      } 
      else if (msg.equals("CMD,STOP")) {
        FSMEvent e = EVENT_STOP;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,STOP");
      }
      else if (msg.equals("CMD,RESUME")) {
        FSMEvent e = EVENT_RESUME;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,RESUME");
      }
      else if(msg.equals("NoRude")) {
        FSMEvent e = EVENT_RAKE_WEED_FOUND;
        xQueueSend(fsmQueue, &e, 0);
        //Serial.println("ACK,RESUME");
      }
      else if (msg.startsWith("ATTACK,")) {
        String indexStr = msg.substring(7); // "ATTACK," tiene 7 caracteres
        int grid_index = indexStr.toInt();

      }
      else {
        //Serial.println("ERROR,UNKNOWN_CMD");
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

void TaskBluetoothCommunication(void *pvParameters) {
  (void) pvParameters;
  String incomingString = "";
  
  // Temporizador para enviar toda la telemetría a un intervalo fijo
  uint32_t lastTelemetrySendTime = 0;
  const uint32_t TELEMETRY_INTERVAL_MS = 1000; // Enviar todo cada 1 segundo

  for (;;) {
    // --- Parte 1: Escuchar comandos (sin cambios) ---
    if (btSerial.available() > 0) {
      char c = btSerial.read();
      if (c == '\n') {
        incomingString.trim(); 
        if (incomingString == "START") {
          //Serial.println("DEBUG: Comando START recibido");
          FSMEvent e = EVENT_NAVIGATE;
          xQueueSend(fsmQueue, &e, 0);
        } else if (incomingString == "STOP") {
          //Serial.println("DEBUG: Comando STOP recibido");
          FSMEvent e = EVENT_STOP;
          xQueueSend(fsmQueue, &e, 0);
        } else if (incomingString.startsWith("X_POS:")) {
          // Extraer el valor después de "YAW:"
          String x_btcmdStr = incomingString.substring(6); // 6 es la longitud de "X_POS:"
          int new_x_btcmd = x_btcmdStr.toInt();
          setTargetYaw(newYaw); // Llamar a nuestra función segura
          //Serial.print("DEBUG: Nuevo Yaw Objetivo establecido a -> ");
          //Serial.println(newYaw);
        } else if (incomingString.startsWith("Y_POS:")) {
          // Extraer el valor después de "YAW:"
          String y_btcmdStr = incomingString.substring(6); // 6 es la longitud de "Y_POS:"
          int new_y_btcmd = y_btcmdStr.toInt();
          setTargetYaw(newYaw); // Llamar a nuestra función segura
          //Serial.print("DEBUG: Nuevo Yaw Objetivo establecido a -> ");
          //Serial.println(newYaw);
        }
        incomingString = "";
      } else {
        incomingString += c;
      }
    }

    // --- Parte 2: Construir y enviar el paquete de telemetría periódicamente ---
    if (millis() - lastTelemetrySendTime > TELEMETRY_INTERVAL_MS) {
      
      SystemState currentState = getState();
      float currentVoltage = getBatteryVoltage();

      String stateStr = SystemStateToString(currentState);
      
      char voltageBuffer[10]; // Buffer para convertir el float del voltaje
      dtostrf(currentVoltage, 4, 2, voltageBuffer); // Formato: 4 caracteres en total, 2 decimales
      String voltageStr = String(voltageBuffer);

      String telemetryPacket = "STATE:" + stateStr + ":BATT:" + voltageStr;

      btSerial.println(telemetryPacket);
      //Serial.println("DEBUG: Enviando Telemetría -> " + telemetryPacket);

      lastTelemetrySendTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}
