#pragma once
#include <Arduino.h>
#include <lora_protocol.h>

class Service {
protected:
    uint8_t _id;
    const char* _name;
    uint8_t _version;
    bool _is_active;

public:
    Service(uint8_t id, const char* name, uint8_t version)
        : _id(id), _name(name), _version(version), _is_active(false) {}
    
    virtual ~Service() {}

    uint8_t getId() const { return _id; }
    const char* getName() const { return _name; }
    uint8_t getVersion() const { return _version; }
    bool isActive() const { return _is_active; }

    // Metodi che ogni sensore dovrà implementare a modo suo
    virtual void init() = 0;
    virtual void loop() = 0;

    // Notifica che la configurazione per questo servizio è stata aggiornata in NVS
    virtual void onConfigChanged() {}
    
    // Funzione che gestisce il comando, popola l'ACK e opzionalmente il RISULTATO
    virtual bool executeCmd(const PayloadCmd* cmd, PayloadAck* out_ack, PayloadCmdResult* out_result) = 0;
};