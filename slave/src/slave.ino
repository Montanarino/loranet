#include <Arduino.h>
#include "lora_protocol.h"
#include "lora_manager.h"
#include "services/service_registry.h" 
#include "services/relay_service.h"
#include "services/sensor_service.h"
#include "services/svc_http.h"
#include "ota_receiver.h"
#include <Preferences.h>

#define SENSOR_PIN 34

#define SLAVE_ID       0x01      
#define SLAVE_NAME     "Sensore_Serra" 
#define FW_MAJOR       1
#define FW_MINOR       0
#define CAPABLE_RELAY  true      
#define RELAY_PIN      25     

LoRaManager loraManager;
OtaReceiver otaReceiver;
HttpService myHttp(SVC_HTTP_SERVER, "Web_Admin", 1);
Preferences preferences;
bool is_registered = false;
uint32_t last_announce_time = 0;
uint8_t announce_retries = 0;
uint32_t last_heartbeat_time = 0; 

// ==========================================
// Creiamo i nostri "Moduli" (Hardware)
// ==========================================
RelayService myRelay(SVC_CORE, "Led_Blu_Test", 1, RELAY_PIN);
SensorService mySensor(SVC_SENSOR, "Sensore_Luce", 1, SENSOR_PIN);


void sendAnnounce();
void sendHeartbeat();

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("\n===========================================");
    Serial.printf("[SLAVE 0x%02X] Avvio del sistema LMP Nodo\n", SLAVE_ID);
    Serial.println("===========================================");

    loraManager.begin();

    // 1. Registriamo e accendiamo tutti i moduli HW!
    serviceRegistry.addService(&myRelay);
    serviceRegistry.addService(&mySensor);
    serviceRegistry.addService(&myHttp);
    serviceRegistry.initAll();

    sendAnnounce();
    last_announce_time = millis();
}

void loop() {
    LmpFrame rxFrame;

    if (loraManager.poll(&rxFrame)) {
        // --- LOGICA MESH FORWARDING ---
        if (rxFrame.header.dst != SLAVE_ID && rxFrame.header.dst != LMP_ADDR_BROADCAST) {
            // È un messaggio per qualcun altro. Possiamo fare da relay?
            if (CAPABLE_RELAY && rxFrame.header.type != MSG_RELAY_FWD) {
                Serial.printf("[MESH] Inoltro pacchetto da 0x%02X a 0x%02X...\n", rxFrame.header.src, rxFrame.header.dst);
                
                PayloadRelay relayPayload;
                memset(&relayPayload, 0, sizeof(PayloadRelay));
                relayPayload.original_src = rxFrame.header.src;
                relayPayload.hop_count = 1; 
                relayPayload.last_relay_id = SLAVE_ID;
                relayPayload.original_type = rxFrame.header.type;
                relayPayload.original_seq = rxFrame.header.seq;
                relayPayload.original_len = rxFrame.header.len;
                memcpy(relayPayload.original_payload, rxFrame.payload, rxFrame.header.len);

                LmpFrame relayFrame;
                uint16_t len = lmp_build_frame(&relayFrame, rxFrame.header.dst, SLAVE_ID, MSG_RELAY_FWD, loraManager.getNextSeq(), &relayPayload, sizeof(PayloadRelay));
                loraManager.sendRaw((uint8_t*)&relayFrame, len);
            }
        }

        if (rxFrame.header.dst == SLAVE_ID || rxFrame.header.dst == LMP_ADDR_BROADCAST) {
            
            // Se è un pacchetto wrappato RELAY_FWD, estraiamo l'originale
            if (rxFrame.header.type == MSG_RELAY_FWD) {
                PayloadRelay* relay = (PayloadRelay*)rxFrame.payload;
                Serial.printf("[MESH] Ricevuto pacchetto inoltrato da 0x%02X (orig: 0x%02X)\n", rxFrame.header.src, relay->original_src);
                rxFrame.header.src = relay->original_src;
                rxFrame.header.type = relay->original_type;
                rxFrame.header.seq = relay->original_seq;
                rxFrame.header.len = relay->original_len;
                memcpy(rxFrame.payload, relay->original_payload, relay->original_len);
            }

            switch (rxFrame.header.type) {
                
                case MSG_CONFIG: {
                    PayloadConfig* cfg = (PayloadConfig*)rxFrame.payload;
                    Serial.printf("[CONFIG] Ricevuta configurazione per Svc: 0x%02X\n", cfg->service_id);
                    
                    preferences.begin("lmp_cfg", false);
                    for (int i=0; i<cfg->param_count; i++) {
                        String key = "s" + String(cfg->service_id) + "_" + String(cfg->params[i].key);
                        preferences.putString(key.c_str(), cfg->params[i].value);
                        Serial.printf("-> Salvato: %s = %s\n", key.c_str(), cfg->params[i].value);
                    }
                    preferences.end();

                    // Notifichiamo il sistema che la configurazione è cambiata
                    serviceRegistry.notifyConfigChanged(cfg->service_id);

                    PayloadConfigAck ack;
                    memset(&ack, 0, sizeof(PayloadConfigAck));
                    ack.service_id = cfg->service_id;
                    ack.status = 0;
                    ack.applied = cfg->param_count;

                    LmpFrame ackFrame;
                    uint16_t len = lmp_build_frame(&ackFrame, rxFrame.header.src, SLAVE_ID, MSG_CONFIG_ACK, loraManager.getNextSeq(), &ack, sizeof(PayloadConfigAck));
                    loraManager.sendRaw((uint8_t*)&ackFrame, len);
                    break;
                }
                
                case MSG_OTA_START: {
                    PayloadOtaAck ack;
                    otaReceiver.handleStart((PayloadOtaStart*)rxFrame.payload, &ack);
                    
                    LmpFrame ackFrame;
                    uint16_t len = lmp_build_frame(&ackFrame, rxFrame.header.src, SLAVE_ID, MSG_OTA_ACK, loraManager.getNextSeq(), &ack, sizeof(PayloadOtaAck));
                    loraManager.sendRaw((uint8_t*)&ackFrame, len);
                    break;
                }

                case MSG_OTA_CHUNK: {
                    PayloadOtaAck ack;
                    otaReceiver.handleChunk((PayloadOtaChunk*)rxFrame.payload, &ack);
                    
                    LmpFrame ackFrame;
                    uint16_t len = lmp_build_frame(&ackFrame, rxFrame.header.src, SLAVE_ID, MSG_OTA_ACK, loraManager.getNextSeq(), &ack, sizeof(PayloadOtaAck));
                    loraManager.sendRaw((uint8_t*)&ackFrame, len);
                    break;
                }

                case MSG_OTA_END: {
                    PayloadOtaAck ack;
                    otaReceiver.handleEnd((PayloadOtaEnd*)rxFrame.payload, &ack);
                    
                    LmpFrame ackFrame;
                    uint16_t len = lmp_build_frame(&ackFrame, rxFrame.header.src, SLAVE_ID, MSG_OTA_ACK, loraManager.getNextSeq(), &ack, sizeof(PayloadOtaAck));
                    loraManager.sendRaw((uint8_t*)&ackFrame, len);
                    break;
                }

                case MSG_ACK: {
                    PayloadAck* ack = (PayloadAck*)rxFrame.payload;
                    if (!is_registered && ack->status == ACK_OK) {
                        is_registered = true;
                        last_heartbeat_time = millis(); 
                        Serial.println("[SYSTEM] Registrazione confermata dal Master!");

                        // ========================================================
                        // NUOVO: Invio automatico della lista servizi al Master!
                        // ========================================================
                        PayloadServiceList svcList;
                        serviceRegistry.buildServiceListPayload(&svcList);

                        LmpFrame listFrame;
                        uint16_t list_len = lmp_build_frame(
                            &listFrame, LMP_ADDR_MASTER, SLAVE_ID, MSG_SERVICE_LIST, 
                            loraManager.getNextSeq(), &svcList, sizeof(PayloadServiceList)
                        );

                        if (list_len > 0) {
                            loraManager.sendRaw((uint8_t*)&listFrame, list_len);
                            Serial.println("-> [TX] Inviata SERVICE_LIST al Master.");
                        }
                    }
                    break;
                }

                case MSG_DISCOVERY_REQ: {
                    delay((SLAVE_ID * 37) % ANNOUNCE_JITTER_MAX); 
                    sendAnnounce();
                    break;
                }

                case MSG_CMD: {
                    if (is_registered) {
                        // 2. SMISTAMENTO AUTOMATICO! Non ci importa quale sia il comando
                        // il Registry troverà il servizio giusto da solo e risponderà!
                        serviceRegistry.dispatchCmd(&rxFrame, &loraManager, SLAVE_ID);
                    }
                    break;
                }

                default: break;
            }
        }
    }

    if (!is_registered) {
        if (millis() - last_announce_time > STANDALONE_RETRY_MS) {
            if (announce_retries < 3) {
                sendAnnounce();
                announce_retries++;
            } else {
                is_registered = true; 
            }
            last_announce_time = millis();
        }
    } else {
        // ESECUZIONE NORMALE 
        if (millis() - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
            sendHeartbeat();
            last_heartbeat_time = millis();
        }
        
        // 3. Facciamo "girare" tutti i sensori
        serviceRegistry.loopAll();
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
    
    // Il numero dei servizi ora è calcolato da solo!
    announcePayload.service_count = serviceRegistry.getCount(); 

    uint16_t frame_len = lmp_build_frame(
        &txFrame, LMP_ADDR_MASTER, SLAVE_ID, MSG_ANNOUNCE, 
        loraManager.getNextSeq(), &announcePayload, sizeof(PayloadAnnounce)
    );

    if (frame_len > 0) loraManager.sendRaw((uint8_t*)&txFrame, frame_len);
}

void sendHeartbeat() {
    LmpFrame txFrame;
    PayloadHeartbeat hbPayload;
    memset(&hbPayload, 0, sizeof(PayloadHeartbeat));

    hbPayload.uptime_s = millis() / 1000;
    hbPayload.free_heap_pct = (ESP.getFreeHeap() * 100) / ESP.getHeapSize(); 
    hbPayload.last_rssi = 0;          
    hbPayload.active_services = 0xFF; // Aggiorneremo la bitmask più avanti
    hbPayload.relay_mode = CAPABLE_RELAY ? 1 : 0;

    uint16_t frame_len = lmp_build_frame(
        &txFrame, LMP_ADDR_MASTER, SLAVE_ID, MSG_HEARTBEAT, 
        loraManager.getNextSeq(), &hbPayload, sizeof(PayloadHeartbeat)
    );

    if (frame_len > 0) loraManager.sendRaw((uint8_t*)&txFrame, frame_len);
}