#include "slave_registry.h"
#include <string.h>

SlaveRegistry::SlaveRegistry() {
    // Inizializza tutto l'array marcando gli slot come liberi e puliti
    for (int i = 0; i < MAX_SLAVES; i++) {
        _nodes[i].is_allocated = false;
        _nodes[i].is_online = false;
        _nodes[i].id = 0;
    }
}

bool SlaveRegistry::addSlave(uint8_t id, const PayloadAnnounce* announce) {
    // Filtro di sicurezza: non possiamo registrare il Master stesso o l'indirizzo di Broadcast
    if (id == LMP_ADDR_MASTER || id == LMP_ADDR_BROADCAST) return false;

    SlaveNode* node = getSlaveById(id);
    
    // Se il nodo non esiste in rubrica, cerchiamo il primo slot di memoria libero
    if (node == nullptr) {
        for (int i = 0; i < MAX_SLAVES; i++) {
            if (!_nodes[i].is_allocated) {
                node = &_nodes[i];
                node->is_allocated = true;
                node->id = id;
                break; // Slot trovato, usciamo dal ciclo
            }
        }
    }

    // Se l'array è completamente pieno e non abbiamo trovato slot liberi, rifiutiamo
    if (node == nullptr) return false;

    // Popoliamo i dati del nodo estraendoli in modo sicuro dal PayloadAnnounce
    strncpy(node->name, announce->name, 16);
    node->name[16] = '\0'; // Terminazione forzata per sicurezza (evita buffer overflow)
    
    node->fw_major = announce->fw_major;
    node->fw_minor = announce->fw_minor;
    
    // Nel protocollo, il bit 0 del campo flags indica se è un ripetitore (0x01)
    node->is_repeater = (announce->flags & 0x01) != 0; 
    node->hop_count = announce->hop;
    node->relay_src = announce->relay_src;
    
    // Registriamo il timestamp attuale e lo dichiariamo online
    node->last_seen = millis();
    node->is_online = true;

    return true;
}

bool SlaveRegistry::updateHeartbeat(uint8_t id, const PayloadHeartbeat* hb) {
    SlaveNode* node = getSlaveById(id);
    
    // Se riceviamo un heartbeat da un nodo "fantasma" (non presente in rubrica), lo ignoriamo.
    // Il nodo dovrà prima o poi fare un ANNOUNCE per farsi conoscere dal Master.
    if (node == nullptr) return false;

    // Aggiorniamo i parametri vitali
    node->last_seen = millis();
    node->is_online = true; // Nel caso fosse andato offline, lo riportiamo in vita
    node->last_rssi = hb->last_rssi;
    node->free_heap_pct = hb->free_heap_pct;
    
    return true;
}

SlaveNode* SlaveRegistry::getSlaveById(uint8_t id) {
    // Scansione lineare dell'array alla ricerca dell'ID corrispondente
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (_nodes[i].is_allocated && _nodes[i].id == id) {
            return &_nodes[i]; // Restituisce il puntatore al nodo trovato
        }
    }
    return nullptr; // Nodo non trovato
}

void SlaveRegistry::checkTimeouts() {
    uint32_t current_time = millis();
    
    for (int i = 0; i < MAX_SLAVES; i++) {
        // Controlliamo solo i nodi attualmente marcati come ONLINE
        if (_nodes[i].is_allocated && _nodes[i].is_online) {
            
            // HEARTBEAT_TIMEOUT_MS è definito nel tuo lora_protocol.h
            // Se è passato troppo tempo dall'ultimo last_seen, il nodo viene dichiarato OFFLINE
            if (current_time - _nodes[i].last_seen > HEARTBEAT_TIMEOUT_MS) {
                _nodes[i].is_online = false;
            }
        }
    }
}

uint8_t SlaveRegistry::getRegisteredCount() const {
    uint8_t count = 0;
    // Conta quanti slot dell'array sono attualmente "occupati"
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (_nodes[i].is_allocated) {
            count++;
        }
    }
    return count;
}