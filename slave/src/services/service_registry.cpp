#include "service_registry.h"

ServiceRegistry serviceRegistry; 

ServiceRegistry::ServiceRegistry() {
    _service_count = 0;
    for (int i = 0; i < MAX_SERVICES; i++) {
        _services[i] = nullptr;
    }
}

bool ServiceRegistry::addService(Service* service) {
    if (_service_count < MAX_SERVICES) {
        _services[_service_count++] = service;
        return true;
    }
    return false;
}

void ServiceRegistry::initAll() {
    for (int i = 0; i < _service_count; i++) {
        if (_services[i] != nullptr) {
            _services[i]->init();
        }
    }
}

void ServiceRegistry::loopAll() {
    for (int i = 0; i < _service_count; i++) {
        if (_services[i] != nullptr && _services[i]->isActive()) {
            _services[i]->loop();
        }
    }
}

void ServiceRegistry::dispatchCmd(const LmpFrame* rxFrame, LoRaManager* lora, uint8_t slave_id) {
    PayloadCmd* cmd = (PayloadCmd*)rxFrame->payload;
    
    Service* target_service = nullptr;
    // 1. Cerca il servizio col corrispondente service_id
    for (int i = 0; i < _service_count; i++) {
        if (_services[i] != nullptr && _services[i]->getId() == cmd->service_id) {
            target_service = _services[i];
            break;
        }
    }

    PayloadAck ack;
    memset(&ack, 0, sizeof(PayloadAck));
    ack.ack_seq = rxFrame->header.seq;

    // 2. Fai eseguire il comando al modulo specifico
    if (target_service != nullptr) {
        target_service->executeCmd(cmd, &ack);
    } else {
        Serial.printf("[WARN] Comando scartato. Servizio 0x%02X non trovato sul nodo.\n", cmd->service_id);
        ack.status = ACK_NOT_FOUND;
        strncpy(ack.message, "NO SVC", 8);
    }

    // 3. Rispondi automaticamente al Master
    LmpFrame ackFrame;
    uint16_t ack_len = lmp_build_frame(
        &ackFrame, rxFrame->header.src, slave_id, MSG_ACK,
        lora->getNextSeq(), &ack, sizeof(PayloadAck)
    );

    if (ack_len > 0) {
        lora->sendRaw((uint8_t*)&ackFrame, ack_len);
        Serial.println("-> [TX] ACK generato dal Registry e inviato al Master.");
    }
}

void ServiceRegistry::buildServiceListPayload(PayloadServiceList* payload) {
    memset(payload, 0, sizeof(PayloadServiceList));
    payload->count = _service_count;
    
    for (int i = 0; i < _service_count; i++) {
        if (_services[i] != nullptr) {
            payload->services[i].service_id = _services[i]->getId();
            strncpy(payload->services[i].name, _services[i]->getName(), 16);
            payload->services[i].version = _services[i]->getVersion();
            payload->services[i].active = _services[i]->isActive() ? 1 : 0;
        }
    }
}