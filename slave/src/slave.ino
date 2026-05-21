#include <Arduino.h>
#include "lora_protocol.h"
//#include "services_ids.h"
#include "lora_manager.h"
// #include "services/service_registry.h" // Includerai questo nella Fase 1.2

// ==========================================
// Configurazione del Nodo (Slave)
// ==========================================
#define SLAVE_ID       0x01      // ID fisso programmato (0x01 - 0xFE)
#define SLAVE_NAME     "Sensore_Serra" 
#define FW_MAJOR       1
#define FW_MINOR       0
#define CAPABLE_RELAY  true      // Questo nodo può fare da ripetitore?

// ==========================================
// Variabili di Stato Globali
// ==========================================
LoRaManager loraManager;
bool is_registered = false;
uint32_t last_announce_time = 0;
uint8_t announce_retries = 0;

// Prototipo della funzione di invio Announce
void sendAnnounce();

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("\n===========================================");
    Serial.printf("[SLAVE 0x%02X] Avvio del sistema LMP Nodo\n", SLAVE_ID);
    Serial.println("===========================================");

    // Inizializza la radio UART e i pin
    loraManager.begin();
    
    // Inizializza i servizi locali (Fase 1.2)
    // serviceRegistry.initAll();

    // Inviamo immediatamente il primo Announce per registrarci al Master
    sendAnnounce();
    last_announce_time = millis();
}

void loop() {
    LmpFrame rxFrame;

    // 1. GESTIONE DELLA RICEZIONE (Polling continuo e non bloccante)
    if (loraManager.poll(&rxFrame)) {
        
        // Verifica che il pacchetto sia destinato a noi o sia un broadcast
        if (rxFrame.header.dst == SLAVE_ID || rxFrame.header.dst == LMP_ADDR_BROADCAST) {
            
            Serial.printf("[RX-HW] Ricevuto MsgType: 0x%02X dal nodo: 0x%02X\n", rxFrame.header.type, rxFrame.header.src);

            // 2. DISPATCHER DEI MESSAGGI
            switch (rxFrame.header.type) {
                
                case MSG_ACK: {
                    PayloadAck* ack = (PayloadAck*)rxFrame.payload;
                    if (!is_registered && ack->status == ACK_OK) {
                        is_registered = true;
                        Serial.println("[SYSTEM] Registrazione confermata dal Master! Rete operativa.");
                    }
                    break;
                }

                case MSG_DISCOVERY_REQ: {
                    // Il master ci sta cercando. 
                    // Il protocollo prevede un jitter casuale di 0-500ms per evitare collisioni.
                    uint32_t jitter_ms = (SLAVE_ID * 37) % ANNOUNCE_JITTER_MAX;
                    delay(jitter_ms); // Piccola pausa calcolata
                    sendAnnounce();
                    break;
                }

                case MSG_CMD: {
                    if (is_registered) {
                        // Elabora comando (Fase 2)
                        // serviceRegistry.dispatchCmd((PayloadCmd*)rxFrame.payload);
                    }
                    break;
                }
                
                // Nota: La gestione dell'ACK automatico (se richiesto dal MSG) 
                // avverrà all'interno del metodo loraManager.poll() o subito dopo il dispatch.
                
                default:
                    break;
            }
        }
    }

    // 3. GESTIONE DELLA MODALITÀ BOOT / STANDALONE
    if (!is_registered) {
        // Se non siamo registrati, riproviamo ogni 5 secondi (STANDALONE_RETRY_MS)
        if (millis() - last_announce_time > STANDALONE_RETRY_MS) {
            
            if (announce_retries < 3) {
                Serial.printf("[SYSTEM] Nessun ACK ricevuto. Retry Announce %d/3...\n", announce_retries + 1);
                sendAnnounce();
                announce_retries++;
            } else {
                // Dopo 3 tentativi, smettiamo di inondare la rete e passiamo in modalità autonoma
                Serial.println("[SYSTEM] Timeout registrazione. Passaggio a Modalità STANDALONE.");
                is_registered = true; // Sblocchiamo l'esecuzione locale
            }
            last_announce_time = millis();
        }
    } else {
        // 4. ESECUZIONE NORMALE (Loop dei servizi, es. lettura sensori - Fase 2)
        // serviceRegistry.loopAll();
    }
}

// ==========================================
// Funzioni di Servizio
// ==========================================
void sendAnnounce() {
    LmpFrame txFrame;
    PayloadAnnounce announcePayload;
    memset(&announcePayload, 0, sizeof(PayloadAnnounce));

    // Popoliamo il payload con i nostri dati
    strncpy(announcePayload.name, SLAVE_NAME, 16);
    announcePayload.fw_major = FW_MAJOR;
    announcePayload.fw_minor = FW_MINOR;
    announcePayload.flags = (CAPABLE_RELAY) ? 0x01 : 0x00;
    announcePayload.hop = 0; // Diretto
    announcePayload.service_count = 1; // Temporaneo, sarà dinamico nella Fase 1.2

    // Costruiamo il frame passando alla funzione condivisa:
    // Destinatario: MASTER (0x00)
    // Mittente: SLAVE_ID
    // Tipo: MSG_ANNOUNCE
    // Sequenza: Incrementata e gestita dal LoRaManager
    uint16_t frame_len = lmp_build_frame(
        &txFrame, 
        LMP_ADDR_MASTER, 
        SLAVE_ID, 
        MSG_ANNOUNCE, 
        loraManager.getNextSeq(), // Metodo che dovrai aggiungere al loraManager
        &announcePayload, 
        sizeof(PayloadAnnounce)
    );

    if (frame_len > 0) {
        // Invia i raw bytes sulla Seriale 2
        // Serial2.write((uint8_t*)&txFrame, frame_len);
        Serial.println("[TX] MSG_ANNOUNCE inviato sulla rete.");
    }
}