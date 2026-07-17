#include "telegram_bot.h"
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include "SystemDefinitions.h"
#include "credentials.h"
#include "wifi_manager.h"

// =================================================================
// ESTADO INTERNO DEL MÓDULO
// =================================================================

static WiFiClientSecure secured_client;
static UniversalTelegramBot* bot = nullptr;
static const char* allowedChatId = nullptr;

static const unsigned long BOT_MTBS = 1000;

// =================================================================
// FUNCIONES FORWARD (definidas en main.cpp)
// =================================================================

extern String SystemStateToString(SystemState state);

// =================================================================
// FUNCIONES AUXILIARES INTERNAS
// =================================================================

static void enviarTecladoCalibracion(const String& chat_id) {
    String keyboardJson = "[";
    keyboardJson += "[{\"text\":\"⏫\", \"callback_data\":\"TILT_UP_FAST\"}],";
    keyboardJson += "[{\"text\":\"🔼\", \"callback_data\":\"TILT_UP\"}],";
    keyboardJson += "[{\"text\":\"⏪\", \"callback_data\":\"PAN_LEFT_FAST\"},";
    keyboardJson += "{\"text\":\"◀️\", \"callback_data\":\"PAN_LEFT\"},";
    keyboardJson += "{\"text\":\"💾\", \"callback_data\":\"SAVE_LIMIT\"},";
    keyboardJson += "{\"text\":\"▶️\", \"callback_data\":\"PAN_RIGHT\"},";
    keyboardJson += "{\"text\":\"⏩\", \"callback_data\":\"PAN_RIGHT_FAST\"}],";
    keyboardJson += "[{\"text\":\"🔽\", \"callback_data\":\"TILT_DOWN\"}],";
    keyboardJson += "[{\"text\":\"⏬\", \"callback_data\":\"TILT_DOWN_FAST\"}]";
    keyboardJson += "]";

    bot->sendMessageWithInlineKeyboard(chat_id, "Modo Calibracion Activado. Usa las flechas para mover el laser:", "", keyboardJson);
}

static void enviarMenu(const String& chat_id) {
    String keyboardJson = "[";
    keyboardJson += "[\"🟢 START\", \"🔴 STOP\"],";
    keyboardJson += "[\"📸 FOTO\", \"🕊️ SIMULAR\"],";
    keyboardJson += "[\"⚙️ CALIB 1\", \"⚙️ CALIB 2\"],";
    keyboardJson += "[\"⚙️ CALIB 3\", \"⚙️ CALIB 4\"],";
    keyboardJson += "[\"👁️ VER 1\", \"👁️ VER 2\"],";
    keyboardJson += "[\"👁️ VER 3\", \"👁️ VER 4\"],";
    keyboardJson += "[\"🌐 CLARO\", \"🏠 FAMILIA\"],";
    keyboardJson += "[\"🕐 HORARIOS\", \"⚙️ CONFIG\"],";
    keyboardJson += "[\"📊 STATUS\", \"❓ HELP\"]";
    keyboardJson += "]";

    bot->sendMessageWithReplyKeyboard(chat_id, "Panel de Control Pigeon Invaders:", "", keyboardJson, true, false, false);
}

// =================================================================
// FUNCIONES DE GUARDADO DE CONFIGURACIÓN
// =================================================================

static void guardarHorario(int index, int startMins, int endMins) {
    preferences.begin("config", false);

    char keyStart[5];
    char keyEnd[5];
    sprintf(keyStart, "st%d", index);
    sprintf(keyEnd, "en%d", index);

    preferences.putInt(keyStart, startMins);
    preferences.putInt(keyEnd, endMins);
    preferences.end();

    horariosMonitor[index].startMins = startMins;
    horariosMonitor[index].endMins = endMins;

    Serial.printf("Horario %d actualizado: %d a %d\n", index + 1, startMins, endMins);
}

static void procesarMovimientoManual(const String& text, const String& chat_id) {
    ManualPosCmd cmd;
    bool movimientoValido = true;

    if (text == "TILT_DOWN") {
        currentTilt -= stepSize;
        cmd.cmdType = 1;
        cmd.value = currentTilt;
    }
    else if (text == "TILT_DOWN_FAST") {
        currentTilt -= stepSizefast;
        cmd.cmdType = 1;
        cmd.value = currentTilt;
    }
    else if (text == "TILT_UP") {
        currentTilt += stepSize;
        cmd.cmdType = 1;
        cmd.value = currentTilt;
    }
    else if (text == "TILT_UP_FAST") {
        currentTilt += stepSizefast;
        cmd.cmdType = 1;
        cmd.value = currentTilt;
    }
    else if (text == "PAN_LEFT") {
        currentPan += stepSize;
        cmd.cmdType = 0;
        cmd.value = currentPan;
    }
    else if (text == "PAN_LEFT_FAST") {
        currentPan += stepSizefast;
        cmd.cmdType = 0;
        cmd.value = currentPan;
    }
    else if (text == "PAN_RIGHT") {
        currentPan -= stepSize;
        cmd.cmdType = 0;
        cmd.value = currentPan;
    }
    else if (text == "PAN_RIGHT_FAST") {
        currentPan -= stepSizefast;
        cmd.cmdType = 0;
        cmd.value = currentPan;
    }
    else if (text == "SAVE_LIMIT") {
        FSMEvent e = EVENT_CONFIRM_POINT;
        xQueueSend(fsmQueue, &e, 0);
        bot->sendMessage(chat_id,
            "Limite guardado para el area " + String(AREA_ACTUAL + 1) + ".");
        return;
    }
    else {
        movimientoValido = false;
    }

    if (!movimientoValido) return;

    if (xQueueSend(manualControlQueue, &cmd, 0) != pdPASS) {
        bot->sendMessage(chat_id, "Cola ocupada. Intente nuevamente.", "");
    }
}

static void guardarIntervalo(int mins) {
    preferences.begin("config", false);
    preferences.putInt("intervalMins", mins);
    preferences.end();

    MONITORING_INTERVAL_MS = mins * 60000;
    Serial.printf("Intervalo actualizado: %d minutos\n", mins);
}

static void guardarSpeed(int ms) {
    preferences.begin("config", false);
    preferences.putInt("Speed", ms);
    preferences.end();

    SPEED_MS = ms;
    Serial.printf("Velocidad actualizada: %d milisegundos\n", ms);
}

// =================================================================
// PROCESAMIENTO DE COMANDOS
// =================================================================

static void procesarComandos(const String& text, const String& chat_id) {

    if (text.indexOf("START") >= 0) {
        FSMEvent e = EVENT_START_COMMAND;
        xQueueSend(fsmQueue, &e, 0);
        bot->sendMessage(chat_id, "Sistema ARMADO y en MONITORING.", "Markdown");
    }
    else if (text.indexOf("STOP") >= 0) {
        FSMEvent e = EVENT_STOP_COMMAND;
        xQueueSend(fsmQueue, &e, 0);
        bot->sendMessage(chat_id, "Sistema DETENIDO.", "");
    }
    else if (text.indexOf("CALIB 1") >= 0) {
        AREA_ACTUAL = 0;
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("CALIB 2") >= 0) {
        AREA_ACTUAL = 1;
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("CALIB 3") >= 0) {
        AREA_ACTUAL = 2;
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("CALIB 4") >= 0) {
        AREA_ACTUAL = 3;
        FSMEvent e = EVENT_ENTER_CALIBRATION;
        xQueueSend(fsmQueue, &e, 0);
        enviarTecladoCalibracion(chat_id);
    }
    else if (text.indexOf("FOTO") >= 0) {
        FSMEvent e = EVENT_TAKE_PICTURE;
        xQueueSend(fsmQueue, &e, 0);
        bot->sendMessage(chat_id, "Capturando foto...", "");
    }
    else if (text.indexOf("SIMULAR") >= 0) {
        FSMEvent e = EVENT_PIGEON_DETECTED;
        xQueueSend(fsmQueue, &e, 0);
        bot->sendMessage(chat_id, "Simulando paloma en el area de vigilancia...", "");
    }
    else if (text.indexOf("CLARO") >= 0) {
        if (wifi_switch(SSID_CLARO, PASS_CLARO)) {
            bot->sendMessage(chat_id, "Conectado exitosamente a la red CLARO.", "");
        } else {
            bot->sendMessage(chat_id, "No se pudo conectar a CLARO. Red anterior restaurada.", "");
        }
    }
    else if (text.indexOf("FAMILIA") >= 0) {
        if (wifi_switch(SSID_FAMILIA, PASS_FAMILIA)) {
            bot->sendMessage(chat_id, "Conectado exitosamente a la red FAMILIA.", "");
        } else {
            bot->sendMessage(chat_id, "No se pudo conectar a FAMILIA. Red anterior restaurada.", "");
        }
    }
    else if (text.indexOf("STATUS") >= 0) {
        SystemState state = getState();
        String estadoStr = SystemStateToString(state);

        String respuesta = "ℹ️ Estado actual del sistema: " + estadoStr + "\n\n";
        respuesta += "⏰ Horarios de Monitoreo:\n";

        for (int i = 0; i < NUM_HORARIOS; i++) {
            int start = horariosMonitor[i].startMins;
            int end = horariosMonitor[i].endMins;

            respuesta += "  [" + String(i + 1) + "] ";

            if (start == 0 && end == 0) {
                respuesta += "Desactivado\n";
            } else {
                int startHH = start / 60;
                int startMM = start % 60;
                int endHH = end / 60;
                int endMM = end % 60;

                char buffer[20];
                sprintf(buffer, "%02d:%02d - %02d:%02d\n", startHH, startMM, endHH, endMM);
                respuesta += String(buffer);
            }
        }

        respuesta += "\n⚙️ Configuracion Extra:\n";
        respuesta += "  - Intervalo: " + String(MONITORING_INTERVAL_MS / 60000) + " minutos\n";
        respuesta += "  - Velocidad: " + String(SPEED_MS) + " milisegundos";

        bot->sendMessage(chat_id, respuesta, "");
    }
    else if (text.indexOf("HELP") >= 0) {
        String ayuda = "Ayuda - Comandos Disponibles:\n\n";
        ayuda += "START - Arma el sistema y comienza vigilancia.\n";
        ayuda += "STOP - Detiene toda actividad y vuelve a Idle.\n";
        ayuda += "FOTO - Toma una foto instantanea del area vigilada.\n";
        ayuda += "SIMULAR - Simula la deteccion de una paloma para pruebas.\n";
        ayuda += "CALIB 1-4 - Entra al modo calibracion para el area 1 a 4.\n";
        ayuda += "VER 1-4 - Muestra un preview del area 1 a 4.\n";
        ayuda += "CLARO - Cambia la conexion Wi-Fi a la red CLARO.\n";
        ayuda += "FAMILIA - Cambia la conexion Wi-Fi a la red FAMILIA.\n";
        ayuda += "STATUS - Muestra el estado actual del sistema.\n\n";
        ayuda += "Para comandos de calibracion, usa los botones de flecha para mover el laser y el boton para guardar los limites.";

        bot->sendMessage(chat_id, ayuda, "");
    }
    else if (text.indexOf("VER 1") >= 0) {
        AREA_ACTUAL = 0;
        if (g_calibMinX[0] == 0 && g_calibMaxX[0] == 0) {
            bot->sendMessage(chat_id, "Area 1 no calibrada aun.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW;
            xQueueSend(fsmQueue, &e, 0);
            bot->sendMessage(chat_id, "Mostrando Area 1...", "");
        }
    }
    else if (text.indexOf("VER 2") >= 0) {
        AREA_ACTUAL = 1;
        if (g_calibMinX[1] == 0 && g_calibMaxX[1] == 0) {
            bot->sendMessage(chat_id, "Area 2 no calibrada aun.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW;
            xQueueSend(fsmQueue, &e, 0);
            bot->sendMessage(chat_id, "Mostrando Area 2...", "");
        }
    }
    else if (text.indexOf("VER 3") >= 0) {
        AREA_ACTUAL = 2;
        if (g_calibMinX[2] == 0 && g_calibMaxX[2] == 0) {
            bot->sendMessage(chat_id, "Area 3 no calibrada aun.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW;
            xQueueSend(fsmQueue, &e, 0);
            bot->sendMessage(chat_id, "Mostrando Area 3...", "");
        }
    }
    else if (text.indexOf("VER 4") >= 0) {
        AREA_ACTUAL = 3;
        if (g_calibMinX[3] == 0 && g_calibMaxX[3] == 0) {
            bot->sendMessage(chat_id, "Area 4 no calibrada aun.", "");
        } else {
            FSMEvent e = EVENT_ENTER_PREVIEW;
            xQueueSend(fsmQueue, &e, 0);
            bot->sendMessage(chat_id, "Mostrando Area 4...", "");
        }
    }
    else if (text.indexOf("HORARIOS") >= 0) {
        String msg = "⏰ Configurar horarios:\n\n";
        msg += "Formato: /horario ID HH:MM HH:MM\n\n";
        msg += "Ejemplo:\n";
        msg += "/horario 1 07:00 18:00\n";
        msg += "/horario 2 08:00 14:00\n";
        msg += "/horario 3 20:00 06:00\n\n";
        msg += "ID: 1, 2 o 3\n";
        msg += "Horas: 00:00 a 23:59\n";
        msg += "Para desactivar: /horario ID 00:00 00:00";
        bot->sendMessage(chat_id, msg, "");
    }
    else if (text.indexOf("CONFIG") >= 0) {
        String msg = "⚙️ Configuracion avanzada:\n\n";
        msg += "Cambiar intervalo: /mins <minutos>\n";
        msg += "Cambiar velocidad: /speed <ms>\n";
        msg += "Configurar horario: /horario ID HH:MM HH:MM";
        bot->sendMessage(chat_id, msg, "");
    }
    else if (text.startsWith("/horario")) {
        int id, hInicio, mInicio, hFin, mFin;

        if (sscanf(text.c_str(), "/horario %d %d:%d %d:%d", &id, &hInicio, &mInicio, &hFin, &mFin) == 5) {
            if (id < 1 || id > NUM_HORARIOS) {
                bot->sendMessage(chat_id, "El ID del horario debe ser 1, 2 o 3.", "");
                return;
            }

            int index = id - 1;
            int startMins = (hInicio * 60) + mInicio;
            int endMins = (hFin * 60) + mFin;

            guardarHorario(index, startMins, endMins);

            String respuesta = "Horario " + String(id) + " de vigilancia actualizado:\n";
            respuesta += "Inicio: " + String(hInicio) + ":" + (mInicio < 10 ? "0" : "") + String(mInicio) + "\n";
            respuesta += "Fin: " + String(hFin) + ":" + (mFin < 10 ? "0" : "") + String(mFin);

            bot->sendMessage(chat_id, respuesta, "");
        } else {
            bot->sendMessage(chat_id, "Formato incorrecto. Usa: /horario ID HH:MM HH:MM (ej. /horario 1 07:00 18:00)", "");
        }
    }
    else if (text.startsWith("/mins")) {
        int intervalmins = 2;

        if (sscanf(text.c_str(), "/mins %d", &intervalmins) == 1) {
            guardarIntervalo(intervalmins);
            bot->sendMessage(chat_id, "Intervalo de vigilancia actualizado.", "");
        } else {
            bot->sendMessage(chat_id, "Formato incorrecto. Usa: /mins <minutos> (ej. /mins 5)", "");
        }
    }
    else if (text.startsWith("/speed")) {
        int velocidad_MS = 50;

        if (sscanf(text.c_str(), "/speed %d", &velocidad_MS) == 1) {
            guardarSpeed(velocidad_MS);
            bot->sendMessage(chat_id, "Velocidad de movimiento actualizada.", "");
        } else {
            bot->sendMessage(chat_id, "Formato incorrecto. Usa: /speed <segundos> (ej. /speed 5)", "");
        }
    }
    else {
        procesarMovimientoManual(text, chat_id);
    }
}

// =================================================================
// MANEJO DE MENSAJES
// =================================================================

static void handleNewMessages(int numNewMessages) {
    for (int i = 0; i < numNewMessages; i++) {
        String chat_id = String(bot->messages[i].chat_id);
        String text = bot->messages[i].text;
        String type = bot->messages[i].type;

        if (chat_id != allowedChatId) {
            bot->sendMessage(chat_id, "Acceso denegado.", "");
            continue;
        }

        if (type == "callback_query") {
            bot->answerCallbackQuery(bot->messages[i].query_id);

            if (getState() == STATE_CALIB_SET_LL || getState() == STATE_CALIB_SET_UR) {
                procesarMovimientoManual(text, chat_id);
            }
            else {
                bot->sendMessage(chat_id, "El modo calibracion no esta activo.", "");
            }
        }
        else if (type == "message") {
            if (text == "/start" || text == "/menu") {
                enviarMenu(chat_id);
            } else {
                procesarComandos(text, chat_id);
            }
        }
    }
}

// =================================================================
// API PÚBLICA
// =================================================================

void telegram_init(const char* botToken, const char* chatId) {
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    bot = new UniversalTelegramBot(botToken, secured_client);
    allowedChatId = chatId;
}

void telegram_sendMessage(const char* chatId, const String& msg, const String& format) {
    if (bot) {
        bot->sendMessage(chatId, msg, format);
    }
}

void telegram_sendMenu(const char* chatId) {
    enviarMenu(chatId);
}

void telegram_task(void* pvParameters) {
    (void)pvParameters;

    while (!wifiSystemReady) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bool firstRun = true;
    Serial.println("Telegram Task: Iniciando polling...");

    for (;;) {
        if (!wifi_reconnect()) {
            continue;
        }

        int numNewMessages = bot->getUpdates(bot->last_message_received + 1);

        while (numNewMessages) {
            handleNewMessages(numNewMessages);
            numNewMessages = bot->getUpdates(bot->last_message_received + 1);
        }

        if (getState() == STATE_IDLE && firstRun) {
            enviarMenu(allowedChatId);
            firstRun = false;
        }

        vTaskDelay(pdMS_TO_TICKS(BOT_MTBS));
    }
}
