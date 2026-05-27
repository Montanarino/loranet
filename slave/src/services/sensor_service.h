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

    bool executeCmd(const PayloadCmd* cmd, PayloadAck* out_ack, PayloadCmdResult* out_result) override {
        // Decidiamo che il comando con ID 0x01 significa "Leggi il valore attuale"
        if (cmd->cmd_id == 0x01) {
            uint16_t valore_letto = analogRead(_pin); // Legge la tensione sul pin (0 - 4095)
            Serial.printf("[SENSOR] Richiesta lettura dal Master. Valore: %d\n", valore_letto);

            // Popoliamo l'ACK positivo
            out_ack->status = ACK_OK;
            strncpy(out_ack->message, "READ OK", 8);

            // Popoliamo il Risultato con i dati grezzi
            out_result->service_id = _id;
            out_result->cmd_id = cmd->cmd_id;
            out_result->status = 0x00; // Successo
            out_result->data_len = 2;
            out_result->data[0] = valore_letto & 0xFF;
            out_result->data[1] = (valore_letto >> 8) & 0xFF;

            return true;
        }
        
        out_ack->status = ACK_UNKNOWN_CMD;
        strncpy(out_ack->message, "ERR CMD", 8);
        return false;
    }
};