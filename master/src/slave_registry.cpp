#include "slave_registry.h"
#include <string.h>

SlaveRegistry::SlaveRegistry() {
    // Inizializza tutto l'array marcando gli slot come liberi
    for (int i = 0; i < MAX_SLAVES; i++) {
        _nodes[i].is_allocated = false;
        _nodes[i].is_online = false;
    }
}

bool SlaveRegistry::addSlave(uint8_t id, const PayloadAnnounce* announce) {
    if (id == LMP_ADDR_MASTER || id == LMP_ADDR_BROADCAST) return false;

    SlaveNode* node = getSlaveById(id);
    
    // Se il nodo non esiste, cerchiamo uno slot libero
    if (node == nullptr) {
        for (int i = 0; i < MAX_SLAVES; i++) {
            if (!_nodes[i].is_allocated) {
                node = &_nodes[i];
                node->is_allocated = true;
                node->id = id;
                break;
            }
        }
    }

    // Se l'array è pieno e non abbiamo trovato slot
    if (node == nullptr) return false;

    // Aggiorniamo/Popoliamo i dati del nodo estratti dall'ANNOUNCE
    strncpy(node->name, announce->name, 16);
    node->name[16] = '\0'; // Garantisce sempre la terminazione della stringa
    
    node->fw_major = announce->fw_major;
    node->fw_minor = announce->fw_minor;
    
    // Il bit 0 del campo flags indica se è un ripetitore
    node->is_repeater = (announce->flags & 0x01) != 0; 
    node->hop_count = announce->hop;
    node->relay_src = announce->relay_src;
    
    // Resettiamo il timer di timeout e lo dichiariamo online
    node->last_seen = millis();
    node->is_online = true;

    return true;
}

bool SlaveRegistry::updateHeartbeat(uint8_t id, const PayloadHeartbeat* hb) {
    SlaveNode* node = getSlaveById(id);
    
    // Se riceviamo un heartbeat da un nodo sconosciuto, lo ignoriamo.
    // Il Master deve prima forzarlo a fare un ANNOUNCE (tramite Discovery).
    if (node == nullptr) return false;

    // Aggiorniamo i parametri vitali
    node->last_seen = millis();
    node->is_online = true; // Se era offline, ora è tornato online
    node->last_rssi = hb->last_rssi;
    node->free_heap_pct = hb->free_heap_pct;
    
    return true;
}

SlaveNode* SlaveRegistry::getSlaveById(uint8_t id) {
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (_nodes[i].is_allocated && _nodes[i].id == id) {
            return &_nodes[i];
        }
    }
    return nullptr;
}

void SlaveRegistry::checkTimeouts() {
    uint32_t current_time = millis();
    
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (_nodes[i].is_allocated && _nodes[i].is_online) {
            
            // HEARTBEAT_TIMEOUT_MS è definito in lora_protocol.h (30000 ms)
            if (current_time - _nodes[i].last_seen > HEARTBEAT_TIMEOUT_MS) {
                _nodes[i].is_online = false;
                
                // NOTA: Qui si potrebbe anche inserire una stampa a console
                // Serial.printf("[REGISTRY] Nodo 0x%02X è andato OFFLINE!\n", _nodes[i].id);
            }
        }
    }
}

uint8_t SlaveRegistry::getRegisteredCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (_nodes[i].is_allocated) {
            count++;
        }
    }
    return count;
}