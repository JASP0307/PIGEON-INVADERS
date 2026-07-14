#ifndef SYSTEM_DEFINITIONS_H
#define SYSTEM_DEFINITIONS_H

#include <Preferences.h>
#include "ServoModule.h"
#include "Laser.h"

// --- PINES EXACTOS PARA FREENOVE ESP32-S3 WROOM CAM ---
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// =================================================================
// 1. ESTADOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
enum SystemState {
  STATE_INITIALIZING,
  STATE_IDLE,
  STATE_MONITORING,
  STATE_PICTURE_PRE_ATTACK,
  STATE_PICTURE_POST_ATTACK,
  STATE_ATTACKING,
  STATE_CALIB_SET_LL,
  STATE_CALIB_SET_UR,
  STATE_CALIB_SAVE,
  STATE_CALIB_PREVIEW,
  STATE_ERROR
};

// =================================================================
// 2. EVENTOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
typedef enum {
    EVENT_NONE,
    EVENT_INIT_COMPLETE,
    EVENT_START_COMMAND,
    EVENT_STOP_COMMAND,
    EVENT_MANUAL_COMMAND,
    EVENT_TIMER_EXPIRED,
    EVENT_TAKE_PICTURE,
    EVENT_PROCESSING_COMPLETE,
    EVENT_PIGEON_DETECTED,
    EVENT_NO_PIGEON,
    EVENT_ATTACK_COMPLETE,
    EVENT_ENTER_CALIBRATION,
    EVENT_ENTER_PREVIEW,
    EVENT_CONFIRM_POINT,
    EVENT_CALIBRATION_DONE,
    EVENT_PREVIEW_DONE,
    EVENT_RESUME
} FSMEvent;

// =================================================================
// 3. TIPOS Y ESTRUCTURAS
// =================================================================

typedef enum {
    PATTERN_RECTANGLE_PREVIEW,
    PATTERN_ZIGZAG_HORIZ,
    PATTERN_ZIGZAG_VERT
} PatternType;

const int AREAS = 4;

typedef struct {
    PatternType currentType;
    bool active;
    int stepIndex;
    int areaPhase;
    int targetArea[AREAS];
    int totalAreas;
    float minX, maxX, minY, maxY;
} PatternContext;

typedef struct {
    int cmdType; // 0 = Set X, 1 = Set Y
    int value;
} ManualPosCmd;

struct RangoHorario {
  int startMins = 0;
  int endMins = 0;
};

const int NUM_HORARIOS = 3;

// =================================================================
// 4. DECLARACIONES EXTERNAS DE VARIABLES GLOBALES
//    (Definidas en main.cpp)
// =================================================================

// Estado de la FSM
extern volatile SystemState currentState;

// Área de calibración actual (0-3)
extern int AREA_ACTUAL;

// Contexto del patrón de movimiento del láser
extern PatternContext patCtx;

// Parámetros de movimiento manual
extern int stepSize;
extern int stepSizefast;

// Intervalo de monitoreo en milisegundos
extern int MONITORING_INTERVAL_MS;

// Datos de calibración de las 4 áreas
extern int g_calibMinX[AREAS];
extern int g_calibMaxX[AREAS];
extern int g_calibMinY[AREAS];
extern int g_calibMaxY[AREAS];

// Posiciones home de las 4 áreas
extern int g_homeX[AREAS];
extern int g_homeY[AREAS];

// Posición manual actual de los servos
extern int currentPan;
extern int currentTilt;

// Horarios de monitoreo
extern RangoHorario horariosMonitor[NUM_HORARIOS];

// Puntos temporales de calibración
extern int temp_X1, temp_Y1;
extern int temp_X2, temp_Y2;

// Bandera para sincronizar el arranque
extern volatile bool wifiSystemReady;

// Objeto NVS para persistencia
extern Preferences preferences;

// Velocidad de movimiento de servos en ms
extern int SPEED_MS;

// =================================================================
// HANDLES FreeRTOS (definidos en main.cpp)
// =================================================================

extern QueueHandle_t fsmQueue;
extern QueueHandle_t manualControlQueue;

// =================================================================
// FUNCIONES GLOBALES (definidas en main.cpp)
// =================================================================

SystemState getState();
void setState(SystemState newState);
String SystemStateToString(SystemState state);

// =================================================================
// OBJETOS GLOBALES DE HARDWARE (definidos en main.cpp)
// =================================================================

extern ServoModule SERVO_X;
extern ServoModule SERVO_Y;
extern Laser Laser_01;

#endif // SYSTEM_DEFINITIONS_H
