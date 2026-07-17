#include "schedule.h"

void cargarHorarios() {
  preferences.begin("config", true);

  for (int i = 0; i < NUM_HORARIOS; i++) {
    char keyStart[5];
    char keyEnd[5];
    sprintf(keyStart, "st%d", i);
    sprintf(keyEnd, "en%d", i);

    int defaultStart = (i == 0) ? 420 : 0;
    int defaultEnd = (i == 0) ? 1080 : 0;

    horariosMonitor[i].startMins = preferences.getInt(keyStart, defaultStart);
    horariosMonitor[i].endMins = preferences.getInt(keyEnd, defaultEnd);
    
    Serial.printf("Boot - Horario %d cargado: %d a %d minutos\n", i + 1, horariosMonitor[i].startMins, horariosMonitor[i].endMins);
  }

  preferences.end();
}

bool isWithinOperatingHours() {
  struct tm timeinfo;
  
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Error: No se pudo obtener la hora para validar el horario.");
    return false; 
  }

  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;
  int currentMins = (currentHour * 60) + currentMinute;

  for (int i = 0; i < NUM_HORARIOS; i++) {
    int start = horariosMonitor[i].startMins;
    int end = horariosMonitor[i].endMins;
    
    if (start == 0 && end == 0) continue; 

    bool enHorario = false;

    if (start <= end) {
      enHorario = (currentMins >= start && currentMins <= end);
    } else {
      enHorario = (currentMins >= start || currentMins <= end);
    }

    if (enHorario) {
      return true;
    }
  }

  return false; 
}
