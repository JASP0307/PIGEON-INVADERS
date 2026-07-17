#ifndef SERVO_PATTERNS_H
#define SERVO_PATTERNS_H

#include <Arduino.h>
#include "SystemDefinitions.h"

#define ZIGZAG_STEP_SIZE 1.0f
#define TARGET_TOLERANCE 0.1f

void startPattern(PatternType type, int min_x, int max_x, int min_y, int max_y);
void stopPattern(int x_home, int y_home);
void updatePatternLogic();

#endif // SERVO_PATTERNS_H
