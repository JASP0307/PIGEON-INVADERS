#ifndef TELEGRAM_BOT_H
#define TELEGRAM_BOT_H

#include <Arduino.h>

// Inicializa el bot de Telegram.
void telegram_init(const char* botToken, const char* allowedChatId);

// Función de tarea FreeRTOS para polling de Telegram.
void telegram_task(void* pvParameters);

// Envía un mensaje de texto a un chat.
void telegram_sendMessage(const char* chatId, const String& msg, const String& format = "");

// Envía el menú principal de control.
void telegram_sendMenu(const char* chatId);

#endif // TELEGRAM_BOT_H
