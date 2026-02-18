// Máquina de Estados para Arduino Mega - Robot de Desmalezado

#include <Arduino.h>
#include <queue.h>
#include <semphr.h>
#include "ServoModule.h"
#include "PinOut.h"
#include "Laser.h"

#define btSerial Serial1

// Handles globales
QueueHandle_t fsmQueue;
QueueHandle_t deltaQueue;
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t batteryMutex;


ServoModule SERV_01(Pinout::BrazoDelta::SERVO_1);
ServoModule SERV_02(Pinout::BrazoDelta::SERVO_2);

Laser Laser_01(Pinout::Laser::Laser_1);


TaskHandle_t xHandleServos;

// Prototipos de tareas
void TaskFSM(void *pvParameters);

void TaskSensors(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);
void TaskBluetoothCommunication(void *pvParameters);

// Funciones auxiliares del brazo delta
void delta_init();
void delta_moveTo(double x, double y, double z);

// Funciones auxiliares
RobotState getState();
void setState(RobotState newState);

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

  // Crear mutex para proteger medición de batería
  batteryMutex = xSemaphoreCreateMutex();
  if (batteryMutex == NULL) {
      Serial.println("Error: No se pudo crear el batteryMutex");
  }


  deltaQueue = xQueueCreate(5, sizeof(int));

  pinMode(Pinout::TiraLED::LEDs, OUTPUT);
  
  
  Serial.println("Motores inicializados");
 
  // Crear tareas
  xTaskCreate(TaskFSM,                    "FSM",        512, NULL, 3, NULL);
  xTaskCreate(TaskSensors,                "Sensors",    256, NULL, 2, NULL);
  xTaskCreate(TaskBattery,                "Battery",    128, NULL, 2, NULL);
  xTaskCreate(TaskComms,                  "Comms", 256, NULL, 2, NULL);
  xTaskCreate(TaskServoControl,           "ServoControl", 256, NULL, 3, &xHandleServos);
  xTaskCreate(TaskBluetoothCommunication, "Bluetooth Test Task", 1024, NULL, 2, NULL);
  xTaskCreate(TaskDeltaControl,           "DeltaControl", 256, NULL, 2, &xHandleDelta);
  
  Serial.println("Tareas FreeRTOS creadas");
}

void loop() {}

// Funciones thread-safe para estado
RobotState getState() {
  RobotState state;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    state = currentState;
    xSemaphoreGive(stateMutex);
  }
  return state;
}

void setState(RobotState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    currentState = newState;
    xSemaphoreGive(stateMutex);
  }
}

String robotStateToString(RobotState state) {
  switch(state) {
    
    case IDLE: return "IDLE";
    case NAVIGATING: return "NAVIGATING";
    case MOVING_TO_WEED: return "MOVING_TO_WEED";
    case LASERING: return "LASERING";
    case RETURNING_HOME: return "RETURNING_HOME";
    case ERROR_STATE: return "ERROR_STATE";
    case LOW_BATTERY: return "LOW_BATTERY";
    case OBSTACLE: return "OBSTACLE";
    default: return "UNKNOWN";
  }
}

float getBatteryVoltage() {
  float voltage;
  if (xSemaphoreTake(batteryMutex, portMAX_DELAY)) {
    voltage = g_batteryVoltage;
    xSemaphoreGive(batteryMutex);
  }
  return voltage;
}


void poblarGrid() {

    grid[19] = { 15.00, -90.00, -90.00 };
    grid[18] = { 5.00, -80.00, -100.00 };
    grid[17] = { 5.00, -80.00, -90.00 };
    grid[16] = { 5.00, -80.00, -75.00 };
    grid[15] = { 5.00, -80.00, -65.00 };
    grid[14] = { -60.00, -45.00, -80.00 };
    grid[13] = { -50.00, -25.00, -100.00 };
    grid[12] = { -50.00, -10.00, -120.00 };
    grid[11] = { -60.00, -0.00, -110.00 };
    grid[10] = { -90.00, 15.00, -65.00 };
    grid[9] = {  00.00, -100.00, -75.00 };
    grid[8] = { -15.00, -100.00, -75.00 };
    grid[7] = { -5.00, -95.00, -70.00 };
    grid[6] = { -10.00, -90.00, -55.00 };
    grid[5] = { -75.00, -55.00, -60.00 };
    grid[4] = { -75.00, -55.00, -60.00 };
    grid[3] = { -55.00, -40.00, -100.00 };
    grid[2] = { -40.00, -35.00, -110.00 };
    grid[1] = { -75.00, -20.00, -100.00 };
    grid[0] = { -90.00, -5.00, -110.00 };

    //Serial.println("Grid de ataque poblado.");
}

void delta_init() {

    SERV_01.begin();
    SERV_02.begin();
    
    // Poblar el grid con tus coordenadas
    poblarGrid();
    
    // Pre-calcular valores para la rotación
    double rotation_rad = ROTATION_ANGLE_DEG * PI / 180.0;
    cos_theta = cos(rotation_rad);
    sin_theta = sin(rotation_rad);

    // Mover a la posición HOME inicial
    delta_moveTo_Compensated(HOME_X, HOME_Y, HOME_Z);
    //Serial.println("Brazo Delta inicializado y en HOME.");
}


void delta_moveTo(double x, double y, double z) {
    if (DK.inverse(x, y, z) == no_error) {
        int s1 = SERVO1_HORIZONTAL - DK.a;
        int s2 = SERVO2_HORIZONTAL - DK.b;
        int s3 = SERVO3_HORIZONTAL - DK.c;
        SERV_01.setTarget(s1);
        SERV_02.setTarget(s2);
        SERV_03.setTarget(s3);
    } else {
        //Serial.println("ERROR: Posicion DELTA inalcanzable.");
    }
}


void onEnterIdle() {
  //Serial.println("Entrando en IDLE");

  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(Pinout::TiraLED::LEDs, HIGH);
}

void onEnterNavigating() {
  Serial.println("START_DETECTION");
  vTaskDelay(pdMS_TO_TICKS(1500)); 
  Serial.println("NAVIGATING");

  digitalWrite(Pinout::TiraLED::LEDs, HIGH);
}

void onEnterLowBattery() {
  //Serial.println("Entrando en LOW_BATTERY");
  digitalWrite(Pinout::TiraLED::LEDs, LOW);
}

void onEnterObstacle() {
  //Serial.println("Entrando en OBSTACLE");
  digitalWrite(Pinout::TiraLED::LEDs, LOW);
}

void onEnterLasering() {
  //Serial.println("Entrando en LASERING (2s)");
  Laser_01.on();
}

void onEnterReturningHome() {
  //Serial.println("Entrando en RETURNING_HOME");
  Laser_01.off();
}

void onEnterMovingToWeed() {
  //Serial.println("Entrando en MOVING_TO_WEED");

// FSM principal
void TaskFSM(void *pvParameters) {
  (void) pvParameters;

  FSMEvent receivedEvent;
  RobotState currentState, newState;

  // Establecer estado inicial y ejecutar su acción de entrada
  setState(IDLE);
  onEnterIdle();

  for (;;) {
    if (xQueueReceive(fsmQueue, &receivedEvent, pdMS_TO_TICKS(100))) {
      RobotState state = getState();
      
      currentState = getState();
      newState = currentState; // Por defecto, no hay cambio de estado

      // Estos eventos pueden ocurrir en cualquier estado y tienen precedencia.
      if (receivedEvent == EVENT_LOW_BATTERY) {
          newState = LOW_BATTERY;
      } else if (receivedEvent == EVENT_ERROR) {
          newState = ERROR_STATE;
      } else {
          // LÓGICA DE TRANSICIÓN: solo decide el siguiente estado
          switch (currentState) {
            case IDLE:
                if (receivedEvent == EVENT_NAVIGATE) newState = NAVIGATING;
                break;

            case NAVIGATING:
                if (receivedEvent == EVENT_STOP) newState = IDLE;
                else if (receivedEvent == EVENT_OBSTACLE) newState = OBSTACLE;
                else if (receivedEvent == EVENT_WEED_FOUND) newState = MOVING_TO_WEED;
                else if (receivedEvent == EVENT_IR_SIGNAL_DETECTED) newState = ROW_CHANGE;
                else if (receivedEvent == EVENT_RAKE_WEED_FOUND) {
                  // Solo iniciamos la acción si no está ya en progreso.
                  if (!g_isRaking) {
                      //Serial.println("RASTRILLO: Iniciando secuencia concurrente.");
                      g_isRaking = true;
                      rakeStartTime = xTaskGetTickCount();
                      // Acción inmediata: Bajar el rastrillo
                      // SERV_04.setTarget(POS_TRABAJO_RASTRILLO); 
                  }
                }
                break;
                
            case MOVING_TO_WEED:
                if (receivedEvent == EVENT_ARM_AT_TARGET) newState = LASERING;
                break;
            
            case LASERING:
                if (receivedEvent == EVENT_LASER_COMPLETE) newState = RETURNING_HOME;
                break;

            case RETURNING_HOME:
                if (receivedEvent == EVENT_ATTACK_COMPLETE) newState = NAVIGATING;
                break;
            
            case OBSTACLE:
                // El evento de fin de espera ahora viene de la cola
                if (receivedEvent == EVENT_TIMEOUT_OBSTACLE) newState = NAVIGATING;
                else if (receivedEvent == EVENT_RESUME) newState = NAVIGATING;
                break;

            case LOW_BATTERY:
            case ERROR_STATE:
                if (receivedEvent == EVENT_RESUME) newState = IDLE;
                break;
            case ROW_CHANGE:
                if (receivedEvent == EVENT_ROW_CHANGED) newState = NAVIGATING;
                break;
          }
        }

      // APLICAR CAMBIO DE ESTADO Y EJECUTAR ACCIÓN DE ENTRADA
      if (newState != currentState) {
          setState(newState);

          switch (newState) {
              case IDLE: onEnterIdle(); break;
              case NAVIGATING: onEnterNavigating(); break;
              case LOW_BATTERY: onEnterLowBattery(); break;
              case OBSTACLE: onEnterObstacle(); break;
              case LASERING: onEnterLasering(); break;
              case RETURNING_HOME: onEnterReturningHome(); break;
              case MOVING_TO_WEED: onEnterMovingToWeed(); break;
              case ROW_CHANGE: onEnterRowChange(); break;
          }
      }
    }

    // GESTIÓN DE TIMEOUTS
    currentState = getState();
    if (currentState == LASERING) {
        if ((xTaskGetTickCount() - laserStartTime) >= pdMS_TO_TICKS(2000)) {
            FSMEvent e = EVENT_LASER_COMPLETE;
            xQueueSend(fsmQueue, &e, 0);
        }
    }
    if (currentState == OBSTACLE) {
        if ((xTaskGetTickCount() - obstacleEntryTime) >= pdMS_TO_TICKS(5000)) {
            FSMEvent e = EVENT_TIMEOUT_OBSTACLE;
            xQueueSend(fsmQueue, &e, 0);
        }
    }

    if (g_isRaking && (xTaskGetTickCount() - g_rakeStartTime >= pdMS_TO_TICKS(5000))) {
            //Serial.println("RASTRILLO: 5 segundos completados. Subiendo rastrillo.");
            //SERV_04.setTarget(POS_INICIAL_RASTRILLO); // Subir el rastrillo
            g_isRaking = false; // Finalizar la secuencia
    }
  
  }
}

void TaskServoControl(void *pvParameters) {
  (void) pvParameters;
  delta_init();
  SERV_04.begin();
  vTaskDelay(pdMS_TO_TICKS(1000));

  for (;;) {
    SERV_01.update();
    SERV_02.update();
    vTaskDelay(pdMS_TO_TICKS(25)); 
  }
}

void TaskDeltaControl(void *pvParameters){
  (void) pvParameters;

  int grid_index;
  FSMEvent eventToSend;

  for(;;){
    // Esperar a que llegue un comando (índice del grid) a la cola
    if(xQueueReceive(deltaQueue, &grid_index, portMAX_DELAY)){

      //   Validar el índice recibido
      if(grid_index >= 0 && grid_index < NUM_GRID_POINTS){
        
        // Notificar a la FSM que el brazo se está moviendo
        eventToSend = EVENT_WEED_FOUND;
        xQueueSend(fsmQueue, &eventToSend, 0);
        
        // Obtener las coordenadas del grid y moverse
        GridPoint target = grid[grid_index];
        delta_moveTo_Compensated(target.x, target.y, target.z);
        vTaskDelay(pdMS_TO_TICKS(1500)); // Simular tiempo de movimiento

        // Notificar a la FSM que el brazo llegó al objetivo
        eventToSend = EVENT_ARM_AT_TARGET;
        xQueueSend(fsmQueue, &eventToSend, 0);

        // --- Aquí se quedaría esperando hasta que termine la acción (ej. láser) ---
        // La FSM se encarga de la temporización del láser.
        // Después de que la FSM complete el estado LASERING, enviará un comando de regreso a casa.

        // Simulación de espera durante el lasering
        vTaskDelay(pdMS_TO_TICKS(3000)); 

        // Volver a la posición HOME
        delta_moveTo_Compensated(HOME_X, HOME_Y, HOME_Z);
        vTaskDelay(pdMS_TO_TICKS(1500)); // Simular tiempo de regreso

        // Notificar a la FSM que la secuencia completa ha terminado
        eventToSend = EVENT_ATTACK_COMPLETE;
        xQueueSend(fsmQueue, &eventToSend, 0);
        Serial.println("DONE");
        
      } else {
        //Serial.println("ERROR: Índice de grid inválido recibido.");
      }
    }
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
        
        // Enviar el índice a la cola del brazo delta
        if (xQueueSend(deltaQueue, &grid_index, pdMS_TO_TICKS(100)) == pdPASS) {
            //Serial.println("ACK,ATTACK_CMD_RECEIVED");
        } else {
            //Serial.println("ERROR,DELTA_QUEUE_FULL");
        }
      }
      else {
        //Serial.println("ERROR,UNKNOWN_CMD");
      }
    }
    
    // Enviar heartbeat periódico usando FreeRTOS ticks
    if ((xTaskGetTickCount() - lastHeartbeat) > pdMS_TO_TICKS(5000)) {
      RobotState state = getState();
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
        } else if (incomingString.startsWith("YAW:")) {
          // Extraer el valor después de "YAW:"
          String yawValueStr = incomingString.substring(4); // 4 es la longitud de "YAW:"
          float newYaw = yawValueStr.toFloat();
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
      
      RobotState currentState = getState();
      float currentVoltage = getBatteryVoltage();

      String stateStr = robotStateToString(currentState);
      
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