#pragma once
#include "service.h"

class RelayService : public Service {
private:
    uint8_t _pin;

public:
    // Passiamo l'ID, il nome, la versione e il pin fisico
    RelayService(uint8_t id, const char* name, uint8_t version, uint8_t pin)
        : Service(id, name, version), _pin(pin) {}

    void init() override {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
        _is_active = true;
        Serial.printf("[SERVICE] %s Inizializzato sul pin %d\n", _name, _pin);
    }

    void loop() override {
        // Il relè non ha operazioni continue da fare nel loop
    }

    bool executeCmd(const PayloadCmd* cmd, PayloadAck* out_ack) override {
        // Se il comando è 0x10 (il nostro comando di accensione inventato)
        if (cmd->cmd_id == 0x10) {
            uint8_t stato_richiesto = cmd->args[0];
            digitalWrite(_pin, stato_richiesto ? HIGH : LOW);
            Serial.printf("[RELAY] Eseguito comando fisico: %s\n", stato_richiesto ? "ON" : "OFF");

            // Popoliamo l'ACK positivo
            out_ack->status = ACK_OK;
            strncpy(out_ack->message, "CMD OK", 8);
            return true;
        }
        
        // Se il comando non è 0x10, restituiamo errore
        out_ack->status = ACK_UNKNOWN_CMD;
        strncpy(out_ack->message, "ERR CMD", 8);
        return false;
    }
};