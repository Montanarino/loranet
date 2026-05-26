#include <Arduino.h>
#include "lora_protocol.h"
#include "lora_manager.h"
#include "slave_registry.h"
#include "config.h"

// =========================================================================
// HANDLES DI FREERTOS
// =========================================================================
QueueHandle_t loraRxQueue = NULL;      
SemaphoreHandle_t registryMutex = NULL; 
SemaphoreHandle_t radioMutex = NULL; // <--- NUOVO MUTEX PER PROTEGGERE IL BUS SPI

LoRaManager   loraManager;
SlaveRegistry slaveRegistry;

void loraRxTask(void *pvParameters);
void mainLogicTask(void *pvParameters);
void consoleTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("\n=======================================================");
    Serial.println("[MASTER] Avvio del Sistema Coordinatore Mesh LMP v1.0");
    Serial.println("=======================================================");

    if (!loraManager.begin()) {
        Serial.println("[CRITICAL] Inizializzazione Hardware LoRa FALLITA!");
        while (1) { delay(1000); }
    }
    Serial.println("[RADIO] Hardware LoRa inizializzato correttamente.");

    loraRxQueue = xQueueCreate(10, sizeof(LmpFrame));
    if (loraRxQueue == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare loraRxQueue!");
        while (1);
    }

    registryMutex = xSemaphoreCreateMutex();
    // Creazione del nuovo Semaforo per la Radio
    radioMutex = xSemaphoreCreateMutex(); 
    
    if (registryMutex == NULL || radioMutex == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare i Mutex!");
        while (1);
    }

    xTaskCreatePinnedToCore(loraRxTask, "LoRa RX Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(mainLogicTask, "Main Logic Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(consoleTask, "Console Task", 4096, NULL, 1, NULL, 1);

    Serial.println("[SYSTEM] Tutti i task FreeRTOS sono attivi. Sistema pronto.");
}

void loop() {
    vTaskDelete(NULL);
}

void loraRxTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame tempFrame;

    for (;;) {
        bool packetReceived = false;

        // PROTEZIONE SPI: Nessuno può interrompere la lettura
        if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
            packetReceived = loraManager.poll(&tempFrame);
            xSemaphoreGive(radioMutex);
        }

        if (packetReceived) {
            if (tempFrame.header.dst == LMP_ADDR_MASTER || tempFrame.header.dst == LMP_ADDR_BROADCAST) {
                if (xQueueSend(loraRxQueue, &tempFrame, pdMS_TO_TICKS(10)) != pdPASS) {
                    Serial.println("[WARN] loraRxQueue satura! Pacchetto radio perso.");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void mainLogicTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame frameToProcess;

    for (;;) {
        if (xQueueReceive(loraRxQueue, &frameToProcess, portMAX_DELAY) == pdPASS) {
            
            switch (frameToProcess.header.type) {
                
                case MSG_ANNOUNCE: {
                    PayloadAnnounce* announce = (PayloadAnnounce*)frameToProcess.payload;
                    Serial.printf("[LOGIC] Ricevuta richiesta ANNOUNCE dal Nodo: %s (ID: 0x%02X)\n", 
                                  announce->name, frameToProcess.header.src);
                    
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        if (slaveRegistry.addSlave(frameToProcess.header.src, announce)) {
                            Serial.printf("-> [REGISTRY] Nodo 0x%02X registrato con successo.\n", frameToProcess.header.src);
                        } 
                        xSemaphoreGive(registryMutex);
                        
                        PayloadAck ackPayload;
                        memset(&ackPayload, 0, sizeof(PayloadAck));
                        ackPayload.ack_seq = frameToProcess.header.seq; 
                        ackPayload.status = ACK_OK;
                        strncpy(ackPayload.message, "REG OK", 8);

                        LmpFrame ackFrame;
                        uint16_t ack_len = lmp_build_frame(
                            &ackFrame, frameToProcess.header.src, LMP_ADDR_MASTER, MSG_ACK,
                            loraManager.getNextSeq(), &ackPayload, sizeof(PayloadAck)
                        );

                        if (ack_len > 0) {
                            // PROTEZIONE SPI: L'invio blocca momentaneamente la ricezione
                            if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
                                loraManager.sendRaw((uint8_t*)&ackFrame, ack_len);
                                xSemaphoreGive(radioMutex);
                            }
                        }
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

                case MSG_ACK: {
                    PayloadAck* ack = (PayloadAck*)frameToProcess.payload;
                    Serial.printf("[LOGIC] Ricevuto ACK dal Nodo 0x%02X. Stato: 0x%02X, Messaggio: '%s'\n", 
                                  frameToProcess.header.src, ack->status, ack->message);
                    break;
                }

                case MSG_SERVICE_LIST: {
                    PayloadServiceList* list = (PayloadServiceList*)frameToProcess.payload;
                    Serial.printf("[LOGIC] Ricevuta SERVICE_LIST dal Nodo 0x%02X.\n", frameToProcess.header.src);
                    
                    // ACQUISIZIONE MUTEX: Salviamo la lista nel registro in modo sicuro!
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        slaveRegistry.updateServices(frameToProcess.header.src, list);
                        xSemaphoreGive(registryMutex);
                        Serial.println("-> [REGISTRY] Lista servizi del nodo aggiornata con successo in memoria.");
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }
}

void consoleTask(void *pvParameters) {
    (void)pvParameters;
    Serial.print("\nLMP-MASTER> ");

    for (;;) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); 

            if (input.length() > 0) {
                
                if (input == "list") {
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
                                              n->id, n->name, n->is_online ? "ONLINE" : "OFFLINE",
                                              n->is_repeater ? "SI" : "NO", ago);
                            }
                        }
                        Serial.println("-------------------------------------------------------");
                        xSemaphoreGive(registryMutex);
                    }
                } 
                else if (input.startsWith("cmd ")) {
                    // Nuova sintassi: cmd <ID_NODO> <ID_SERVIZIO> <ID_COMANDO> <ARGOMENTO>
                    int t_id, s_id, c_id, a_val;
                    
                    // Analizza la stringa per estrarre i 4 numeri separati da spazio
                    if (sscanf(input.c_str(), "cmd %d %d %d %d", &t_id, &s_id, &c_id, &a_val) == 4) {
                        
                        PayloadCmd cmdPayload;
                        memset(&cmdPayload, 0, sizeof(PayloadCmd));
                        cmdPayload.service_id = (uint8_t)s_id; 
                        cmdPayload.cmd_id = (uint8_t)c_id;         
                        cmdPayload.args_len = 1;
                        cmdPayload.args[0] = (uint8_t)a_val;       

                        LmpFrame cmdFrame;
                        uint16_t cmd_len = lmp_build_frame(
                            &cmdFrame, (uint8_t)t_id, LMP_ADDR_MASTER, MSG_CMD,
                            loraManager.getNextSeq(), &cmdPayload, sizeof(PayloadCmd)
                        );

                        if (cmd_len > 0) {
                            if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
                                loraManager.sendRaw((uint8_t*)&cmdFrame, cmd_len);
                                xSemaphoreGive(radioMutex);
                            }
                            Serial.printf("[TX] Comando inviato! Nodo:0x%02X, Svc:0x%02X, Cmd:0x%02X, Arg:%d\n", 
                                          t_id, s_id, c_id, a_val);
                        }
                    } else {
                        Serial.println("Errore. Sintassi: cmd <NODO> <SERVIZIO> <COMANDO> <ARG> (Tutti numerici)");
                        Serial.println("Esempio Relè ON   : cmd 1 0 16 1  (Svc 0, Cmd 16 (0x10), Arg 1)");
                        Serial.println("Esempio Leggi Luce: cmd 1 1 1 0   (Svc 1, Cmd 1, Arg 0)");
                    }
                }
                else if (input == "help") {
                    Serial.println("\nComandi CLI Disponibili:");
                    Serial.println("  list  - Mostra lo stato di tutti i nodi slave censiti");
                    Serial.println("  info  - Lista dei servizi");
                    Serial.println("  cmd   - Invia comando: cmd <ID_NODO> <1|0> (es. cmd 1 1 accende il LED)");
                    Serial.println("  help  - Mostra questo menu");
                } 
                else if (input.startsWith("info ")) {
                    uint8_t target_id = (uint8_t)input.substring(5).toInt();
                    
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        SlaveNode* n = slaveRegistry.getSlaveById(target_id);
                        if (n != nullptr) {
                            Serial.printf("\n--- INFO NODO 0x%02X (%s) ---\n", n->id, n->name);
                            Serial.printf("Moduli HW Installati (%d):\n", n->service_count);
                            for (int i = 0; i < n->service_count; i++) {
                                Serial.printf(" [%d] ID: 0x%02X | %-15s | Ver: %d | Stato: %s\n",
                                              i+1, n->services[i].service_id, n->services[i].name, 
                                              n->services[i].version, n->services[i].active ? "ON" : "OFF");
                            }
                            Serial.println("---------------------------------");
                        } else {
                            Serial.printf("Errore: Nodo 0x%02X non trovato nel registro.\n", target_id);
                        }
                        xSemaphoreGive(registryMutex);
                    }
                }
                else {
                    Serial.println("Comando sconosciuto. Digita 'help' per la lista.");
                }
            }
            Serial.print("LMP-MASTER> ");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}