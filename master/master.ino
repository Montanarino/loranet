#include <Arduino.h>
#include "shared/lora_protocol.h"
#include "shared/service_ids.h"
#include "src/lora_manager.h"
#include "src/slave_registry.h"

// ==========================================
// Handles di FreeRTOS (Specifiche Fase 1)
// ==========================================
QueueHandle_t loraRxQueue = NULL;      // Coda per passare i frame validati da RX alla Logica
SemaphoreHandle_t registryMutex = NULL; // Mutex per proteggere l'accesso alla SlaveRegistry

// Istanza del gestore LoRa
LoRaManager loraManager;
SlaveRegistry slaveRegistry;

// Prototipi dei Task FreeRTOS
void loraRxTask(void *pvParameters);
void mainLogicTask(void *pvParameters);
void consoleTask(void *pvParameters);

void setup() {
    // Inizializzazione della console di debug principale dell'ESP32
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("\n===========================================");
    Serial.println("[MASTER] Avvio del sistema LMP Coordinatore");
    Serial.println("===========================================");

    // 1. Inizializzazione Hardware del modulo LoRa (comandi AT)
    loraManager.begin();

    // 2. Creazione della coda FreeRTOS (alloca spazio per 10 frame LmpFrame)
    loraRxQueue = xQueueCreate(10, sizeof(LmpFrame));
    if (loraRxQueue == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare loraRxQueue!");
        while (1); // Blocco di sicurezza
    }

    // 3. Creazione del Mutex per la protezione dei dati condivisi
    registryMutex = xSemaphoreCreateMutex();
    if (registryMutex == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare registryMutex!");
        while (1);
    }

    // 4. Creazione dei Task con affinità di Core e priorità dedicate (Fase 1)
    
    // loraRxTask: Core 0, Priorità 2 (Alta priorità per svuotare l'UART ed evitare overflow)
    xTaskCreatePinnedToCore(
        loraRxTask,         // Funzione del task
        "LoRa RX Task",     // Nome testuale (per debug)
        4096,               // Stack size in byte
        NULL,               // Parametri in ingresso
        2,                  // Priorità (2 = Alta)
        NULL,               // Task handle (non necessario)
        0                   // ID del CORE (Core 0)
    );

    // mainLogicTask: Core 0, Priorità 1 (Processa la logica di rete con calma)
    xTaskCreatePinnedToCore(
        mainLogicTask,
        "Main Logic Task",
        4096,
        NULL,
        1,                  // Priorità (1 = Media/Bassa)
        NULL,
        0                   // ID del CORE (Core 0)
    );

    // consoleTask: Core 1, Priorità 1 (CLI utente isolata sull'altro core hardware)
    xTaskCreatePinnedToCore(
        consoleTask,
        "Console Task",
        4096,
        NULL,
        1,                  // Priorità (1 = Media/Bassa)
        NULL,
        1                   // ID del CORE (Core 1)
    );

    Serial.println("[MASTER] Tutti i task FreeRTOS sono stati avviati correttamente.");
}

void loop() {
    // Nell'architettura FreeRTOS su ESP32, loop() gira nativamente come task sul Core 1.
    // Poiché abbiamo delegato tutto a task espliciti, eliminiamo questo task per risparmiare risorse.
    vTaskDelete(NULL);
}

// =========================================================================
// TASK 1: loraRxTask (Core 0, Priorità 2)
// =========================================================================
void loraRxTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame tempFrame;

    Serial.println("[TASK] loraRxTask avviato sul Core 0.");

    for (;;) {
        // Interroghiamo la macchina a stati non bloccante del LoRaManager
        if (loraManager.poll(&tempFrame)) {
            Serial.printf("[RX-HW] Catturato frame integro da 0x%02X diretto a 0x%02X (Tipo: 0x%02X)\n", 
                          tempFrame.header.src, tempFrame.header.dst, tempFrame.header.type);
            
            // Spediamo il frame nella coda per il task logico.
            // Se la coda è temporaneamente piena, attendiamo al massimo 10ms prima di scartarlo.
            if (xQueueSend(loraRxQueue, &tempFrame, pdMS_TO_TICKS(10)) != pdPASS) {
                Serial.println("[WARN] loraRxQueue satura! Frame radio perso.");
            }
        }

        // Rilasciamo brevemente la CPU (1 millisecondo) per evitare lo 'starvation' 
        // del Core 0 e permettere al microcontrollore di gestire i processi di background (Wi-Fi/BT/Watchdog)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =========================================================================
// TASK 2: mainLogicTask (Core 0, Priorità 1)
// =========================================================================
void mainLogicTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame frameToProcess;

    Serial.println("[TASK] mainLogicTask avviato sul Core 0.");

    for (;;) {
        // Restiamo in attesa bloccante (0% CPU utilizzata) finché non compare un elemento nella coda
        if (xQueueReceive(loraRxQueue, &frameToProcess, portMAX_DELAY) == pdPASS) {
            
            // Elaborazione semantica basata sul tipo di messaggio (MsgType)
            switch (frameToProcess.header.type) {
                
                case MSG_ANNOUNCE: {
                    // Mappiamo il puntatore del payload generico sulla struct specifica
                    PayloadAnnounce* announce = (PayloadAnnounce*)frameToProcess.payload;
                    
                    Serial.printf("[LOGIC] Elaborazione ANNOUNCE da slave: %s (ID: 0x%02X)\n", announce->name, frameToProcess.header.src);
                    
                    // ACQUISIZIONE MUTEX: Entriamo nella sezione critica per toccare il registro
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        
                        // ZONA SICURA: Qui andranno le chiamate a slave_registry.cpp (Fase 1)
                        // Es: slaveRegistry.addSlave(frameToProcess.header.src, announce);
                        
                        Serial.printf("-> [REGISTRY] Nodo 0x%02X aggiunto con successo. Versione FW: %d.%d\n", 
                                      frameToProcess.header.src, announce->fw_major, announce->fw_minor);
                        
                        // RILASCIO MUTEX: Usciamo immediatamente dalla sezione critica
                        xSemaphoreGive(registryMutex);
                        
                        // NOTA: Qui (nella Fase 1 avanzata) dovrai mandare il MSG_ACK di risposta allo slave
                    } else {
                        Serial.println("[ERROR] Impossibile accedere alla registry: Mutex bloccato!");
                    }
                    break;
                }

                case MSG_HEARTBEAT: {
                    // Logica di aggiornamento heartbeat (Fase 2)
                    break;
                }

                default:
                    Serial.printf("[LOGIC] Avviso: Ricevuto MsgType 0x%02X non ancora implementato.\n", frameToProcess.header.type);
                    break;
            }
        }
    }
}

// =========================================================================
// TASK 3: consoleTask (Core 1, Priorità 1)
// =========================================================================
void consoleTask(void *pvParameters) {
    (void)pvParameters;
    Serial.println("[TASK] consoleTask (CLI) avviato sul Core 1.");
    
    // Stampa del prompt dei comandi iniziale
    Serial.print("\nLMP-MASTER> ");

    for (;;) {
        // Controllo della presenza di comandi scritti dall'utente sul monitor seriale
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Pulisce spazi vuoti o ritorni a capo (\r)

            if (input.length() > 0) {
                
                if (input == "list") {
                    // Richiediamo il Mutex per leggere la tabella in modo coerente
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        
                        Serial.println("\n--- TABELLA NODI SLAVE DISPONIBILI ---");
                        Serial.println("ID   | Nome Nodo       | Stato");
                        Serial.println("-------------------------------------");
                        // Mock temporaneo in attesa dello sviluppo completo di slave_registry.cpp
                        Serial.println("0x01 | Nodo_Test_01    | ONLINE (Mock)"); 
                        Serial.println("-------------------------------------");
                        
                        xSemaphoreGive(registryMutex);
                    } else {
                        Serial.println("Errore: Registro occupato dal sistema. Riprovare.");
                    }
                } 
                else if (input == "help") {
                    Serial.println("\nComandi CLI disponibili:");
                    Serial.println("  list  - Mostra l'elenco degli slave registrati");
                    Serial.println("  help  - Mostra questo menu di aiuto");
                } 
                else {
                    Serial.printf("Comando sconosciuto: '%s'. Digita 'help' per la lista.\n", input.c_str());
                }
            }
            Serial.print("LMP-MASTER> ");
        }

        // Essendo un task di interfaccia utente a polling, un delay di 50ms 
        // è impercettibile all'occhio umano ma scarica del tutto il Core 1
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
