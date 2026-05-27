#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>
#include "slave_registry.h"
#include "lora_manager.h"

/**
 * @brief Inizializza la console seriale e prepara il prompt.
 */
void console_init();

/**
 * @brief Task FreeRTOS che gestisce l'interazione CLI via Serial.
 */
void consoleTask(void *pvParameters);

/**
 * @brief Funzione di utilità per stampare il prompt.
 */
void console_print_prompt();

#endif // CONSOLE_H
