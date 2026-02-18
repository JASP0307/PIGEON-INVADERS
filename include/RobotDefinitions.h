#ifndef ROBOT_DEFINITIONS_H
#define ROBOT_DEFINITIONS_H

// Este archivo centraliza todas las definiciones lógicas, constantes de comportamiento,
// y enumeraciones para la Máquina de Estados Finitos (FSM) del robot.

// =================================================================
// 1. ESTADOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
enum RobotState {
  IDLE,
  LASERING,
  RETURNING_HOME,
  ERROR_STATE,
  OBSTACLE,
};

// =================================================================
// 2. EVENTOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
typedef enum {
  EVENT_NONE,
  EVENT_NAVIGATE,
  EVENT_STOP,
  EVENT_OBSTACLE, 
  EVENT_ARM_AT_TARGET,
  EVENT_LASER_COMPLETE,
  EVENT_ARM_AT_HOME,
  EVENT_LOW_BATTERY,
  EVENT_ERROR,
  EVENT_RESUME,
  EVENT_ATTACK_COMPLETE,
  EVENT_TIMEOUT_OBSTACLE,
  EVENT_IR_SIGNAL_DETECTED,
  EVENT_ROW_CHANGED
} FSMEvent;

enum ArmCommand {
  CMD_IDLE,
  CMD_MOVE_TO_TARGET,
  CMD_RETURN_HOME
};

// =================================================================
// 3. CONSTANTES DE COMPORTAMIENTO
// =================================================================

// Estado protegido por mutex
volatile RobotState currentState = IDLE;

// --- Tiempos de Espera (en milisegundos) ---
const int LASER_ON_TIME_MS = 2500;


// --- LÓGICA DEL BRAZO DELTA ---
// Parámetros físicos y de compensación

const int SERVO1_HORIZONTAL = 172; 
const int SERVO2_HORIZONTAL = 168;

// Estructura y array para el grid de ataque
struct GridPoint { double x; double y; double z;};
const int NUM_GRID_POINTS = 20;
GridPoint grid[NUM_GRID_POINTS];

#endif // ROBOT_DEFINITIONS_H