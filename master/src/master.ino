#include <Arduino.h>
#include "lora_protocol.h"
#include "lora_manager.h"
#include "slave_registry.h"
#include "config.h"

// =========================================================================
// HANDLES DI FREERTOS (Specifiche Architetturali)
// =========================================================================
QueueHandle_t loraRxQueue = NULL;      // Coda per il passaggio sicuro dei frame dal task RX alla logica
SemaphoreHandle_t registryMutex = NULL; // Mutex per proteggere la SlaveRegistry dagli accessi concorrenti

// =========================================================================
// ISTANZE GLOBALI
// =========================================================================
LoRaManager   loraManager;
SlaveRegistry slaveRegistry;

// =========================================================================
// PROTOTIPI DEI TASK FREERTOS
// =========================================================================
void loraRxTask(void *pvParameters);
void mainLogicTask(void *pvParameters);
void consoleTask(void *pvParameters);

// =========================================================================
// SETUP DEL COORDINATORE (MASTER)
// =========================================================================
void setup() {
    // Inizializzazione del monitor seriale di debug dell'ESP32
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("\n=======================================================");
    Serial.println("[MASTER] Avvio del Sistema Coordinatore Mesh LMP v1.0");
    Serial.println("=======================================================");

    // 1. Inizializzazione dello strato fisico radio (UART o SPI tramite config.h)
    if (!loraManager.begin()) {
        Serial.println("[CRITICAL] Inizializzazione Hardware LoRa FALLITA!");
        while (1) { delay(1000); }
    }
    Serial.println("[RADIO] Hardware LoRa inizializzato correttamente.");

    // 2. Creazione della coda FreeRTOS (alloca spazio sicuro in RAM per 10 pacchetti interi)
    loraRxQueue = xQueueCreate(10, sizeof(LmpFrame));
    if (loraRxQueue == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare loraRxQueue!");
        while (1);
    }

    // 3. Creazione del Mutex per la sincronizzazione tra Main Logic e CLI
    registryMutex = xSemaphoreCreateMutex();
    if (registryMutex == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare registryMutex!");
        while (1);
    }

    // 4. Generazione dei Task con affinità di Core e priorità dedicate (Fase 1/2)
    
    // loraRxTask: CORE 0, Priorità 2 (Alta priorità per svuotare i registri hardware ed evitare overflow)
    xTaskCreatePinnedToCore(
        loraRxTask,         // Funzione del task
        "LoRa RX Task",     // Nome identificativo (debug)
        4096,               // Stack size allocato (byte)
        NULL,               // Parametri di input
        2,                  // Livello priorità (2 = Alta)
        NULL,               // Task Handle
        0                   // CORE ID (Core 0 hardware)
    );

    // mainLogicTask: CORE 0, Priorità 1 (Processa la logica semantica di rete a priorità media)
    xTaskCreatePinnedToCore(
        mainLogicTask,
        "Main Logic Task",
        4096,
        NULL,               // Parametri di input
        1,                  // Livello priorità (1 = Media)
        NULL,               // Task Handle
        0                   // CORE ID (Core 0 hardware)
    );

    // consoleTask: CORE 1, Priorità 1 (La CLI utente gira isolata sull'altro core per non pesare sulla radio)
    xTaskCreatePinnedToCore(
        consoleTask,
        "Console Task",
        4096,
        NULL,               // Parametri di input
        1,                  // Livello priorità (1 = Media)
        NULL,               // Task Handle
        1                   // CORE ID (Core 1 hardware)
    );

    Serial.println("[SYSTEM] Tutti i task FreeRTOS sono attivi. Sistema pronto.");
}

// =========================================================================
// LOOP ARDUINO NATIVO
// =========================================================================
void loop() {
    // Nativamente su ESP32, la funzione loop() è un task in esecuzione sul Core 1.
    // Avendo implementato un'architettura a task espliciti, eliminiamo questo thread
    // per liberare risorse e far respirare l'IDLE task del sistema operativo.
    vTaskDelete(NULL);
}

// =========================================================================
// TASK 1: loraRxTask (Core 0, Priorità 2) - Gestione Hardware Asincrona
// =========================================================================
void loraRxTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame tempFrame;

    Serial.println("[TASK] loraRxTask avviato sul Core 0.");

    for (;;) {
        // Interroghiamo a ciclo continuo la macchina a stati non bloccante
        if (loraManager.poll(&tempFrame)) {
            
            // Il protocollo stabilisce che il Master riceve pacchetti diretti a 0x00 o in Broadcast
            if (tempFrame.header.dst == LMP_ADDR_MASTER || tempFrame.header.dst == LMP_ADDR_BROADCAST) {
                
                Serial.printf("[RX-HW] Catturato pacchetto integro da 0x%02X diretto a 0x%02X (Tipo: 0x%02X, SEQ: %d)\n", 
                              tempFrame.header.src, tempFrame.header.dst, tempFrame.header.type, tempFrame.header.seq);
                
                // Spingiamo il frame decodificato nella coda protetta.
                // Se la coda è temporaneamente satura, attendiamo al massimo 10ms prima di scartarlo.
                if (xQueueSend(loraRxQueue, &tempFrame, pdMS_TO_TICKS(10)) != pdPASS) {
                    Serial.println("[WARN] loraRxQueue satura! Pacchetto radio perso.");
                }
            }
        }

        // Rilasciamo la CPU per 1 millisecondo ad ogni ciclo. Evita lo 'starvation' del Core 0 
        // e permette all'ESP32 di gestire i processi vitali di background (Wi-Fi, BT o il Watchdog)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =========================================================================
// TASK 2: mainLogicTask (Core 0, Priorità 1) - Elaborazione e Handshake
// =========================================================================
void mainLogicTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame frameToProcess;

    Serial.println("[TASK] mainLogicTask avviato sul Core 0.");

    for (;;) {
        // Il task si addormenta a consumo 0% CPU finché non appare un elemento nella coda
        if (xQueueReceive(loraRxQueue, &frameToProcess, portMAX_DELAY) == pdPASS) {
            
            // Dispatcher semantico basato sul tipo di messaggio del protocollo (MsgType)
            switch (frameToProcess.header.type) {
                
                case MSG_ANNOUNCE: {
                    // Mappiamo il puntatore del payload generico sulla struct specifica dell'Announce
                    PayloadAnnounce* announce = (PayloadAnnounce*)frameToProcess.payload;
                    
                    Serial.printf("[LOGIC] Ricevuta richiesta ANNOUNCE dal Nodo: %s (ID: 0x%02X)\n", 
                                  announce->name, frameToProcess.header.src);
                    
                    // ACQUISIZIONE MUTEX: Sezione Critica per aggiornare l'array del registro
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        
                        // Memorizzazione sicura dei dati del nodo
                        if (slaveRegistry.addSlave(frameToProcess.header.src, announce)) {
                            Serial.printf("-> [REGISTRY] Nodo 0x%02X registrato con successo. FW Version: %d.%d\n", 
                                          frameToProcess.header.src, announce->fw_major, announce->fw_minor);
                        } else {
                            Serial.println("-> [REGISTRY] Errore: Registro pieno o ID non valido!");
                        }
                        
                        // RILASCIO MUTEX: Usciamo immediatamente dalla sezione critica
                        xSemaphoreGive(registryMutex);
                        
                        // --- COSTRUZIONE E INVIO DEL PACCHETTO DI MSG_ACK (Risposta automatica) ---
                        PayloadAck ackPayload;
                        memset(&ackPayload, 0, sizeof(PayloadAck));
                        
                        // Il protocollo impone: l'ACK deve contenere il SEQ del messaggio a cui risponde
                        ackPayload.ack_seq = frameToProcess.header.seq; 
                        ackPayload.status = ACK_OK;
                        strncpy(ackPayload.message, "REG OK", 8);

                        LmpFrame ackFrame;
                        // Costruiamo il frame strutturato calcolando dinamicamente il nuovo CRC
                        uint16_t ack_len = lmp_build_frame(
                            &ackFrame, 
                            frameToProcess.header.src, // Destinatario: lo slave che ha inviato l'Announce
                            LMP_ADDR_MASTER,           // Mittente: noi stessi (0x00)
                            MSG_ACK,                   // Tipo messaggio
                            frameToProcess.header.seq, // L'ACK usa lo stesso SEQ del frame ricevuto
                            &ackPayload, 
                            sizeof(PayloadAck)
                        );

                        if (ack_len > 0) {
                            // Invio fisico sulla rete radio tramite l'astrazione hardware
                            loraManager.sendFrame(&ackFrame);
                            Serial.printf("-> [TX] Trasmesso MSG_ACK di conferma al Nodo 0x%02X\n", frameToProcess.header.src);
                        } else {
                            Serial.println("[ERROR] Impossibile costruire il frame di ACK.");
                        }
                    } else {
                        Serial.println("[ERROR] Registro occupato. Impossibile elaborare l'Announce.");
                    }
                    break;
                }

                case MSG_HEARTBEAT: {
                    PayloadHeartbeat* hb = (PayloadHeartbeat*)frameToProcess.payload;
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        slaveRegistry.updateHeartbeat(frameToProcess.header.src, hb);
                        xSemaphoreGive(registryMutex);
                    }
                    break;
                }

                default:
                    Serial.printf("[LOGIC] Avviso: Ricevuto MsgType 0x%02X non gestito dal Master.\n", frameToProcess.header.type);
                    break;
            }
        }
    }
}

// =========================================================================
// TASK 3: consoleTask (Core 1, Priorità 1) - Interfaccia CLI Utente
// =========================================================================
void consoleTask(void *pvParameters) {
    (void)pvParameters;
    Serial.println("[TASK] consoleTask (CLI) avviato sul Core 1.");
    
    // Prompt dei comandi iniziale
    Serial.print("\nLMP-MASTER> ");

    for (;;) {
        // Controllo non bloccante della presenza di stringhe inviate sul monitor seriale
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Pulizia da spazi vuoti e caratteri di ritorno a capo (\r)

            if (input.length() > 0) {
                
                if (input == "list") {
                    // Richiediamo il Mutex per leggere la tabella in RAM senza che il task radio la sovrascriva
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        
                        uint8_t count = slaveRegistry.getRegisteredCount();
                        Serial.printf("\n--- TABELLA DISPOSITIVI LMP (Totale: %d) ---\n", count);
                        Serial.println("ID   | Nome Dispositivo | Stato   | Relay? | Last Seen");
                        Serial.println("-------------------------------------------------------");
                        
                        for (uint16_t i = 1; i <= 254; i++) {
                            SlaveNode* n = slaveRegistry.getSlaveById(i);
                            if (n != nullptr) {
                                uint32_t ago = (millis() - n->last_seen) / 1000;
                                Serial.printf("0x%02X | %-16s | %-7s | %-6s | %ds fa\n",
                                              n->id, n->name, 
                                              n->is_online ? "ONLINE" : "OFFLINE",
                                              n->is_repeater ? "SI" : "NO", ago);
                            }
                        }
                        Serial.println("-------------------------------------------------------");
                        
                        xSemaphoreGive(registryMutex);
                    } else {
                        Serial.println("Errore: Registro occupato dal sistema. Riprova.");
                    }
                } 
                else if (input == "help") {
                    Serial.println("\nComandi CLI Disponibili:");
                    Serial.println("  list  - Mostra lo stato di tutti i nodi slave censiti nella rete");
                    Serial.println("  help  - Mostra questo menu di aiuto");
                } 
                else {
                    Serial.printf("Comando sconosciuto: '%s'. Digita 'help' per la lista.\n", input.c_str());
                }
            }
            Serial.print("LMP-MASTER> ");
        }

        // Delay di 50ms per far riposare il Core 1 tra una lettura seriale e l'altra
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}