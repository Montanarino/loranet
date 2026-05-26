#pragma once
#include "service.h"

class SensorService : public Service {
private:
    uint8_t _pin;

public:
    // Costruttore
    SensorService(uint8_t id, const char* name, uint8_t version, uint8_t pin)
        : Service(id, name, version), _pin(pin) {}

    void init() override {
        pinMode(_pin, INPUT);
        _is_active = true;
        Serial.printf("[SERVICE] %s Inizializzato sul pin %d\n", _name, _pin);
    }

    void loop() override {
        // Qui potremmo inserire logiche per leggere il sensore ogni X secondi
        // e far scattare allarmi in automatico, ma per ora lo leggiamo solo "a comando".
    }

    bool executeCmd(const PayloadCmd* cmd, PayloadAck* out_ack) override {
        // Decidiamo che il comando con ID 0x01 significa "Leggi il valore attuale"
        if (cmd->cmd_id == 0x01) {
            uint16_t valore_letto = analogRead(_pin); // Legge la tensione sul pin (0 - 4095)
            Serial.printf("[SENSOR] Richiesta lettura dal Master. Valore: %d\n", valore_letto);

            // Popoliamo l'ACK inserendo il valore letto direttamente nel messaggio testuale!
            out_ack->status = ACK_OK;
            snprintf(out_ack->message, 8, "%d", valore_letto); 
            return true;
        }
        
        out_ack->status = ACK_UNKNOWN_CMD;
        strncpy(out_ack->message, "ERR CMD", 8);
        return false;
    }
};