#include <Arduino.h>
#include "lora_protocol.h"
#include "lora_manager.h"

// ==========================================
// Configurazione del Nodo (Slave)
// ==========================================
#define SLAVE_ID       0x01      
#define SLAVE_NAME     "Sensore_Serra" 
#define FW_MAJOR       1
#define FW_MINOR       0
#define CAPABLE_RELAY  true      

#define RELAY_PIN      25         // <--- Il Pin del LED o Relè da azionare

// ==========================================
// Variabili di Stato Globali
// ==========================================
LoRaManager loraManager;
bool is_registered = false;
uint32_t last_announce_time = 0;
uint8_t announce_retries = 0;
uint32_t last_heartbeat_time = 0; 

void sendAnnounce();
void sendHeartbeat();

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("\n===========================================");
    Serial.printf("[SLAVE 0x%02X] Avvio del sistema LMP Nodo\n", SLAVE_ID);
    Serial.println("===========================================");

    // Inizializziamo il pin hardware come spento
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    loraManager.begin();

    sendAnnounce();
    last_announce_time = millis();
}

void loop() {
    LmpFrame rxFrame;

    if (loraManager.poll(&rxFrame)) {
        if (rxFrame.header.dst == SLAVE_ID || rxFrame.header.dst == LMP_ADDR_BROADCAST) {
            
            switch (rxFrame.header.type) {
                
                case MSG_ACK: {
                    PayloadAck* ack = (PayloadAck*)rxFrame.payload;
                    if (!is_registered && ack->status == ACK_OK) {
                        is_registered = true;
                        last_heartbeat_time = millis(); 
                        Serial.println("[SYSTEM] Registrazione confermata dal Master! Rete operativa.");
                    }
                    break;
                }

                case MSG_DISCOVERY_REQ: {
                    uint32_t jitter_ms = (SLAVE_ID * 37) % ANNOUNCE_JITTER_MAX;
                    delay(jitter_ms); 
                    sendAnnounce();
                    break;
                }

                // ========================================================
                // NUOVA LOGICA: Ricezione comando dal Master (Fase 2)
                // ========================================================
                case MSG_CMD: {
                    if (is_registered) {
                        PayloadCmd* cmd = (PayloadCmd*)rxFrame.payload;
                        
                        // Verifichiamo che il comando sia per noi (Svc: 0x00, Cmd: 0x10)
                        if (cmd->service_id == SVC_CORE && cmd->cmd_id == 0x10) {
                            
                            uint8_t stato_richiesto = cmd->args[0]; // 1 per accendere, 0 per spegnere
                            digitalWrite(RELAY_PIN, stato_richiesto ? HIGH : LOW);
                            
                            Serial.printf("-> [AZIONE] Comando fisico eseguito: Relay %s\n", stato_richiesto ? "ON" : "OFF");

                            // Rispondiamo al Master con un ACK di conferma
                            PayloadAck ackPayload;
                            memset(&ackPayload, 0, sizeof(PayloadAck));
                            ackPayload.ack_seq = rxFrame.header.seq;
                            ackPayload.status = ACK_OK;
                            strncpy(ackPayload.message, "CMD OK", 8);

                            LmpFrame ackFrame;
                            uint16_t ack_len = lmp_build_frame(
                                &ackFrame, rxFrame.header.src, SLAVE_ID, MSG_ACK,
                                loraManager.getNextSeq(), &ackPayload, sizeof(PayloadAck)
                            );
                            
                            if (ack_len > 0) {
                                loraManager.sendRaw((uint8_t*)&ackFrame, ack_len);
                                Serial.println("-> [TX] Inviato ACK di conferma al Master.");
                            }
                        }
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }

    if (!is_registered) {
        if (millis() - last_announce_time > STANDALONE_RETRY_MS) {
            if (announce_retries < 3) {
                Serial.printf("[SYSTEM] Nessun ACK ricevuto. Retry Announce %d/3...\n", announce_retries + 1);
                sendAnnounce();
                announce_retries++;
            } else {
                Serial.println("[SYSTEM] Timeout registrazione. Passaggio a Modalità STANDALONE.");
                is_registered = true; 
            }
            last_announce_time = millis();
        }
    } else {
        if (millis() - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
            sendHeartbeat();
            last_heartbeat_time = millis();
        }
    }
}

void sendAnnounce() {
    LmpFrame txFrame;
    PayloadAnnounce announcePayload;
    memset(&announcePayload, 0, sizeof(PayloadAnnounce));

    strncpy(announcePayload.name, SLAVE_NAME, 16);
    announcePayload.fw_major = FW_MAJOR;
    announcePayload.fw_minor = FW_MINOR;
    announcePayload.flags = (CAPABLE_RELAY) ? 0x01 : 0x00;
    announcePayload.hop = 0; 
    announcePayload.service_count = 1;

    uint16_t frame_len = lmp_build_frame(
        &txFrame, LMP_ADDR_MASTER, SLAVE_ID, MSG_ANNOUNCE, 
        loraManager.getNextSeq(), &announcePayload, sizeof(PayloadAnnounce)
    );

    if (frame_len > 0) {
        loraManager.sendRaw((uint8_t*)&txFrame, frame_len);
        Serial.println("[TX] MSG_ANNOUNCE inviato sulla rete.");
    }
}

void sendHeartbeat() {
    LmpFrame txFrame;
    PayloadHeartbeat hbPayload;
    memset(&hbPayload, 0, sizeof(PayloadHeartbeat));

    hbPayload.uptime_s = millis() / 1000;
    hbPayload.free_heap_pct = (ESP.getFreeHeap() * 100) / ESP.getHeapSize(); 
    hbPayload.last_rssi = 0;          
    hbPayload.active_services = 0x01; 
    hbPayload.relay_mode = CAPABLE_RELAY ? 1 : 0;

    uint16_t frame_len = lmp_build_frame(
        &txFrame, LMP_ADDR_MASTER, SLAVE_ID, MSG_HEARTBEAT, 
        loraManager.getNextSeq(), &hbPayload, sizeof(PayloadHeartbeat)
    );

    if (frame_len > 0) {
        loraManager.sendRaw((uint8_t*)&txFrame, frame_len);
    }
}