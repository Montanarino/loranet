#pragma once
#include "service.h"
#include "../lora_manager.h" 

#define MAX_SERVICES 8

class ServiceRegistry {
private:
    Service* _services[MAX_SERVICES];
    uint8_t _service_count;

public:
    ServiceRegistry();
    bool addService(Service* service);
    void initAll();
    void loopAll();
    
    // Lo smistatore di pacchetti (dispatcher)
    void dispatchCmd(const LmpFrame* rxFrame, LoRaManager* lora, uint8_t slave_id);
    
    uint8_t getCount() const { return _service_count; }
};

extern ServiceRegistry serviceRegistry; // Dichiariamo un'istanza globale