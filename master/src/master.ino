#include <Arduino.h>
#include "lora_protocol.h"
#include "lora_manager.h"
#include "slave_registry.h"
#include "config.h"
#include "console.h"
#include "ota_sender.h"

// =========================================================================
// HANDLES DI FREERTOS
// =========================================================================
QueueHandle_t loraRxQueue = NULL;      
SemaphoreHandle_t registryMutex = NULL; 
SemaphoreHandle_t radioMutex = NULL; 

LoRaManager   loraManager;
SlaveRegistry slaveRegistry;
OtaSender     otaSender;

void loraRxTask(void *pvParameters);
void mainLogicTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("\n=======================================================");
    Serial.println("[MASTER] Avvio del Sistema Coordinatore Mesh LMP v1.0");
    Serial.println("=======================================================");

    if (!LittleFS.begin(true)) {
        Serial.println("[WARN] LittleFS non montato. Formattazione in corso...");
    }

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
    radioMutex = xSemaphoreCreateMutex(); 
    
    if (registryMutex == NULL || radioMutex == NULL) {
        Serial.println("[CRITICAL] Errore: Impossibile creare i Mutex!");
        while (1);
    }

    // Inizializza la console (configurazione e prompt)
    console_init();

    xTaskCreatePinnedToCore(loraRxTask, "LoRa RX Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(mainLogicTask, "Main Logic Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(consoleTask, "Console Task", 4096, NULL, 1, NULL, 1);

    Serial.println("[SYSTEM] Tutti i task FreeRTOS sono attivi. Sistema pronto.");
}

void loop() {
    // In un'app FreeRTOS su ESP32, il loop può essere eliminato
    vTaskDelete(NULL);
}

void loraRxTask(void *pvParameters) {
    (void)pvParameters;
    LmpFrame tempFrame;

    for (;;) {
        bool packetReceived = false;

        // PROTEZIONE SPI
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
    uint32_t lastDiscoveryTime = 0;

    for (;;) {
        // 1. Gestione Discovery Periodico (ogni 30 secondi)
        if (millis() - lastDiscoveryTime > 30000) {
            LmpFrame discFrame;
            uint16_t disc_len = lmp_build_frame(
                &discFrame, LMP_ADDR_BROADCAST, LMP_ADDR_MASTER, MSG_DISCOVERY_REQ,
                loraManager.getNextSeq(), NULL, 0
            );
            
            if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                loraManager.sendRaw((uint8_t*)&discFrame, disc_len);
                xSemaphoreGive(radioMutex);
                // Serial.println("[LOGIC] Inviato DISCOVERY_REQ in broadcast.");
            }
            lastDiscoveryTime = millis();
        }

        // 2. Controllo Timeout Heartbeat (Offline detection)
        if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (uint16_t i = 1; i <= 254; i++) {
                SlaveNode* n = slaveRegistry.getSlaveById(i);
                if (n != nullptr && n->is_online) {
                    if (millis() - n->last_seen > HEARTBEAT_TIMEOUT_MS) {
                        n->is_online = false;
                        Serial.printf("\n[WARN] Nodo 0x%02X (%s) è OFFLINE (timeout heartbeat).\n", n->id, n->name);
                        console_print_prompt();
                    }
                }
            }
            xSemaphoreGive(registryMutex);
        }

        // 3. Elaborazione Messaggi in Coda
        if (xQueueReceive(loraRxQueue, &frameToProcess, pdMS_TO_TICKS(100)) == pdPASS) {
            
            switch (frameToProcess.header.type) {
                
                case MSG_ANNOUNCE: {
                    PayloadAnnounce* announce = (PayloadAnnounce*)frameToProcess.payload;
                    Serial.printf("\n[LOGIC] Ricevuto ANNOUNCE dal Nodo: %s (ID: 0x%02X)\n", 
                                  announce->name, frameToProcess.header.src);
                    
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        if (slaveRegistry.addSlave(frameToProcess.header.src, announce)) {
                            Serial.printf("-> [REGISTRY] Nodo 0x%02X registrato/aggiornato.\n", frameToProcess.header.src);
                        } 
                        xSemaphoreGive(registryMutex);
                        
                        // Risposta con ACK
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

                        if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
                            loraManager.sendRaw((uint8_t*)&ackFrame, ack_len);
                            xSemaphoreGive(radioMutex);
                        }
                    }
                    console_print_prompt();
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
                    Serial.printf("\n[LOGIC] ACK da Nodo 0x%02X. Status: 0x%02X, Msg: '%s'\n", 
                                  frameToProcess.header.src, ack->status, ack->message);
                    console_print_prompt();
                    break;
                }

                case MSG_SERVICE_LIST: {
                    PayloadServiceList* list = (PayloadServiceList*)frameToProcess.payload;
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        slaveRegistry.updateServices(frameToProcess.header.src, list);
                        xSemaphoreGive(registryMutex);
                        Serial.printf("\n[LOGIC] Servizi aggiornati per Nodo 0x%02X.\n", frameToProcess.header.src);
                        console_print_prompt();
                    }
                    break;
                }

                case MSG_CMD_RESULT: {
                    PayloadCmdResult* res = (PayloadCmdResult*)frameToProcess.payload;
                    Serial.printf("\n[LOGIC] RISULTATO COMANDO da 0x%02X (Svc: 0x%02X, Cmd: 0x%02X). Status: 0x%02X\n",
                                  frameToProcess.header.src, res->service_id, res->cmd_id, res->status);
                    if (res->data_len > 0) {
                        Serial.print("-> Dati: ");
                        for (int i=0; i<res->data_len; i++) Serial.printf("%02X ", res->data[i]);
                        Serial.println();
                    }
                    console_print_prompt();
                    break;
                }

                case MSG_OTA_ACK: {
                    PayloadOtaAck* ack = (PayloadOtaAck*)frameToProcess.payload;
                    otaSender.handleOtaAck(ack);
                    break;
                }

                case MSG_CONFIG_ACK: {
                    PayloadConfigAck* ack = (PayloadConfigAck*)frameToProcess.payload;
                    Serial.printf("\n[LOGIC] CONFIG_ACK da Nodo 0x%02X. Svc: 0x%02X, Applicati: %d\n", 
                                  frameToProcess.header.src, ack->service_id, ack->applied);
                    console_print_prompt();
                    break;
                }

                default:
                    break;
            }
        }
        
        otaSender.process();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
