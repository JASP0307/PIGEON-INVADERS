#ifndef SYSTEM_DEFINITIONS_H
#define SYSTEM_DEFINITIONS_H

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

#define MONITORING_INTERVAL_MS (10 * 60 * 1000) // 5 minutos en milisegundos

// Este archivo centraliza todas las definiciones lógicas, constantes de comportamiento,
// y enumeraciones para la Máquina de Estados Finitos (FSM) del robot.

// =================================================================
// 1. ESTADOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
enum SystemState {
  STATE_INITIALIZING,      // Encendido, conexión WiFi y Telegram
  STATE_IDLE,              // Esperando comando Start (Láser apagado)
  STATE_MONITORING,        // Vigilancia intermitente (Loop de 5 min)
  STATE_TAKING_PICTURE,    // Tomando foto y procesando evidencia
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
    EVENT_TAKE_PICTURE,          // Usuario solicita tomar foto manualmente
    EVENT_PROCESSING_COMPLETE,   // Terminó de procesar la foto
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

int g_calibMinX = 0, g_calibMaxX = 100, g_calibMinY = 0, g_calibMaxY = 100, stepSize = 2;
int currentPan = 90;  
int currentTilt = 90;
bool movido = false;

typedef struct {
    int cmdType; // 0 = Set X, 1 = Set Y
    int value;   // El valor del ángulo/posición
} ManualPosCmd;

typedef struct {
  camera_fb_t* fotoAntes;
  camera_fb_t* fotoDespues;
  unsigned long timestamp;
} EvidenciaAtaque;

enum class BotState {
  IDLE,           // Sistema detenido
  RUNNING,        // Vigilando activamente
  CALIBRATING_1,  // Moviendo a esquina INF-IZQ
  CALIBRATING_2,  // Moviendo a esquina SUP-DER
  CONFIRMING_CAL  // Mostrando resumen para confirmar
};

// Estado global del bot
struct BotContext {
  BotState state = BotState::IDLE;
  String   activeChatId = "";
  int      msgIdToEdit  = -1;   // ID del mensaje con el teclado activo
  int      panAngle     = 90;   // Posición actual servo pan
  int      tiltAngle    = 90;   // Posición actual servo tilt
  int      cal1Pan      = -1;   // Límite guardado esquina 1
  int      cal1Tilt     = -1;
  int      cal2Pan      = -1;   // Límite guardado esquina 2
  int      cal2Tilt     = -1;
};

extern BotContext botCtx;

int temp_X1 = 0, temp_Y1 = 100;
int temp_X2 = 100, temp_Y2 = 0;

// Bandera para sincronizar el arranque
volatile bool wifiSystemReady = false;
// "resize_keyboard": true -> Hace que los botones tengan un tamaño normal y no ocupen media pantalla
// "one_time_keyboard": false -> El teclado se queda ahí siempre, no desaparece al pulsarlo
const String keyboardJson = "{\"keyboard\":[[\"/start\", \"/stop\"],[\"/status\"]], \"resize_keyboard\":true, \"one_time_keyboard\":true}";

#endif // SYSTEM_DEFINITIONS_H
