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
  STATE_CALIB_PREVIEW      // Calibración: Dibujando recuadro para confirmación visual
};

// =================================================================
// 2. EVENTOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
typedef enum {
    EVENT_NONE,                  // Sin evento
    EVENT_INIT_COMPLETE,         // WiFi y Telegram listos
    EVENT_START_COMMAND,         // Usuario envía "Start"
    EVENT_STOP_COMMAND,          // Usuario envía "Stop"
    EVENT_TIMER_EXPIRED,         // Pasaron los 5 minutos de espera
    EVENT_PIGEON_DETECTED,       // Visión artificial confirma paloma
    EVENT_NO_PIGEON,             // Visión artificial confirma zona limpia
    EVENT_ATTACK_COMPLETE,       // Terminó el patrón de servos y reporte
    EVENT_ENTER_CALIBRATION,     // Usuario solicita modo calibración
    EVENT_CONFIRM_POINT,         // Usuario confirma un punto (LL o UR)
    EVENT_CALIBRATION_DONE       // Usuario confirma que el recuadro es correcto
} FSMEvent;

// =================================================================
// 3. CONSTANTES DE COMPORTAMIENTO
// =================================================================

// Estado protegido por mutex
volatile SystemState currentState = IDLE;


// --- LÓGICA DEL BRAZO DELTA ---
// Parámetros físicos y de compensación

const int SERVO_HORIZONTAL = 172; 
const int SERVO_VERTICAL = 168;

// Estructura y array para el grid de ataque
struct GridPoint { double x; double y; double z;};
const int NUM_GRID_POINTS = 20;
GridPoint grid[NUM_GRID_POINTS];

#endif // SYSTEM_DEFINITIONS_H
