#include "servo_patterns.h"

static bool hasReachedTarget() {
    float dist01 = abs(SERVO_X.getPosition() - SERVO_X.getTarget());
    float dist02 = abs(SERVO_Y.getPosition() - SERVO_Y.getTarget());
    return (dist01 < TARGET_TOLERANCE && dist02 < TARGET_TOLERANCE);
}

void startPattern(PatternType type, int min_x, int max_x, int min_y, int max_y) {
    patCtx.currentType = type;
    patCtx.minX = min_x; patCtx.maxX = max_x;
    patCtx.minY = min_y; patCtx.maxY = max_y;
    
    patCtx.stepIndex = 0;
    patCtx.active = true;
    
    SERVO_X.setTarget(min_x);
    SERVO_Y.setTarget(min_y);
}

void stopPattern(int x_home, int y_home) {
    patCtx.active = false;
    SERVO_X.setTarget(x_home);
    SERVO_Y.setTarget(y_home);
}

void updatePatternLogic() {
    if (!patCtx.active) return;
    
    if (hasReachedTarget()) {
        float nextX, nextY;
        switch (patCtx.currentType) {
            
            case PATTERN_RECTANGLE_PREVIEW:
                switch (patCtx.stepIndex) {
                    case 0: SERVO_X.setTarget(patCtx.maxX); SERVO_Y.setTarget(patCtx.minY); break;
                    case 1: SERVO_X.setTarget(patCtx.maxX); SERVO_Y.setTarget(patCtx.maxY); break;
                    case 2: SERVO_X.setTarget(patCtx.minX); SERVO_Y.setTarget(patCtx.maxY); break;
                    case 3: SERVO_X.setTarget(patCtx.minX); SERVO_Y.setTarget(patCtx.minY); break;
                    case 4: 
                        patCtx.active = false;
                        { FSMEvent e = EVENT_PREVIEW_DONE;
                        xQueueSend(fsmQueue, &e, 0); }
                        return; 
                }
                patCtx.stepIndex++;
                break;

            case PATTERN_ZIGZAG_HORIZ:
                nextY = patCtx.minY + (patCtx.stepIndex * ZIGZAG_STEP_SIZE);
                
                if (nextY > patCtx.maxY) {
                  SERVO_X.setTarget(patCtx.minX);
                  SERVO_Y.setTarget(patCtx.minY);
                  patCtx.active = false;

                  patCtx.areaPhase++; 

                  if (patCtx.areaPhase < patCtx.totalAreas) {
                      int siguienteArea = patCtx.targetArea[patCtx.areaPhase];
                      Serial.printf("Área %d completa, iniciando Área %d...\n", patCtx.areaPhase, patCtx.areaPhase + 1);
                      startPattern(patCtx.currentType,
                          g_calibMinX[siguienteArea],
                          g_calibMaxX[siguienteArea],
                          g_calibMinY[siguienteArea],
                          g_calibMaxY[siguienteArea]
                      );
                  } else {
                      Serial.println("Todas las áreas asignadas fueron atacadas. Ataque completo.");
                      { FSMEvent e = EVENT_ATTACK_COMPLETE;
                      xQueueSend(fsmQueue, &e, 0); }
                  }
                  
              } else {
                  float targetX = (patCtx.stepIndex % 2 == 0) ? patCtx.maxX : patCtx.minX;
                  SERVO_X.setTarget(targetX);
                  SERVO_Y.setTarget(nextY);
                  patCtx.stepIndex++;
              }
              break;

            case PATTERN_ZIGZAG_VERT:
              nextX = patCtx.minX + (patCtx.stepIndex * ZIGZAG_STEP_SIZE);
              
              if (nextX > patCtx.maxX) {
                  SERVO_X.setTarget(patCtx.minX);
                  SERVO_Y.setTarget(patCtx.minY);
                  patCtx.active = false;

                  patCtx.areaPhase++; 

                  if (patCtx.areaPhase < patCtx.totalAreas) {
                      int siguienteArea = patCtx.targetArea[patCtx.areaPhase];
                      Serial.printf("Área %d completa, iniciando Área %d...\n", patCtx.areaPhase, patCtx.areaPhase + 1);
                      startPattern(patCtx.currentType,
                          g_calibMinX[siguienteArea],
                          g_calibMaxX[siguienteArea],
                          g_calibMinY[siguienteArea],
                          g_calibMaxY[siguienteArea]
                      );
                  } else {
                      Serial.println("Todas las áreas asignadas fueron atacadas. Ataque completo.");
                      { FSMEvent e = EVENT_ATTACK_COMPLETE;
                      xQueueSend(fsmQueue, &e, 0); }
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
