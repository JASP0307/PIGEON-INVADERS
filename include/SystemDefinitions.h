#ifndef SYSTEM_DEFINITIONS_H
#define SYSTEM_DEFINITIONS_H

// Este archivo centraliza todas las definiciones lógicas, constantes de comportamiento,
// y enumeraciones para la Máquina de Estados Finitos (FSM) del robot.

// =================================================================
// 1. ESTADOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
enum SystemState {
  STATE_INITIALIZING,      // Encendido, conexión WiFi y Telegram
  STATE_IDLE,              // Esperando comando Start (Láser apagado)
  STATE_MONITORING,        // Vigilancia intermitente (Loop de 5 min)
  STATE_ATTACKING,         // Láser ON, moviendo servos, reporte
  STATE_CALIB_SET_LL,      // Calibración: Esperando definir Límite Inferior Izquierdo
  STATE_CALIB_SET_UR,      // Calibración: Esperando definir Límite Superior Derecho
  STATE_CALIB_PREVIEW,      // Calibración: Dibujando recuadro para confirmación visual
  STATE_ERROR
};

// =================================================================
// 2. EVENTOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
typedef enum {
    EVENT_NONE,                  // Sin evento
    EVENT_INIT_COMPLETE,         // WiFi y Telegram listos
    EVENT_START_COMMAND,         // Usuario envía "Start"
    EVENT_STOP_COMMAND,          // Usuario envía "Stop"
    EVENT_MANUAL_COMMAND,        // Usuario envía "Comando de ataque"
    EVENT_TIMER_EXPIRED,         // Pasaron los 5 minutos de espera
    EVENT_PIGEON_DETECTED,       // Visión artificial confirma paloma
    EVENT_NO_PIGEON,             // Visión artificial confirma zona limpia
    EVENT_ATTACK_COMPLETE,       // Terminó el patrón de servos y reporte
    EVENT_ENTER_CALIBRATION,     // Usuario solicita modo calibración
    EVENT_CONFIRM_POINT,         // Usuario confirma un punto (LL o UR)
    EVENT_CALIBRATION_DONE,       // Usuario confirma que el recuadro es correcto
    EVENT_RESUME
} FSMEvent;

// =================================================================
// 3. DEFINIENDO PATRONES DE MOVIMIENTO LÁSER
// =================================================================

// Estado protegido por mutex
volatile SystemState currentState = STATE_INITIALIZING;

typedef enum {
    PATTERN_NONE,
    PATTERN_RECTANGLE_PREVIEW,
    PATTERN_ZIGZAG_HORIZ,
    PATTERN_ZIGZAG_VERT
} PatternType;

typedef struct {
    PatternType currentType;
    bool active;
    int stepIndex;       // En qué paso del patrón vamos
    int direction;       // 1 o -1 (para saber si vamos o venimos en el zigzag)
    float currentX;      // Posición actual X calculada
    float currentY;      // Posición actual Y calculada
    // Límites locales copiados de la calibración global
    float minX, maxX, minY, maxY; 
} PatternContext;

PatternContext patCtx;

int g_calibMinX, g_calibMaxX, g_calibMinY, g_calibMaxY;


typedef struct {
    int cmdType; // 0 = Set X, 1 = Set Y
    int value;   // El valor del ángulo/posición
} ManualPosCmd;

float temp_X1, temp_Y1;
float temp_X2, temp_Y2;

// Bandera para sincronizar el arranque
volatile bool wifiSystemReady = false;

#endif // SYSTEM_DEFINITIONS_H
