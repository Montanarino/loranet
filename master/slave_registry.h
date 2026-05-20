#pragma once

#include <Arduino.h>
#include "shared/lora_protocol.h"

// Numero massimo di nodi supportati contemporaneamente in RAM
#define MAX_SLAVES 32 

// Struttura che rappresenta lo stato di un singolo nodo nella rete
struct SlaveNode {
    uint8_t  id;                // Indirizzo LoRa dello slave (0x01 - 0xFE)
    char     name[17];          // Nome human-readable (16 char + terminatore)
    uint32_t last_seen;         // Timestamp (millis) dell'ultimo contatto
    bool     is_online;         // true se il nodo sta rispondendo
    bool     is_repeater;       // true se il nodo ha la capacità di fare da relay
    uint8_t  hop_count;         // 0 = diretto, >0 = inoltrato da un relay
    uint8_t  relay_src;         // ID del nodo che ha inoltrato il pacchetto
    
    // Info di sistema
    uint8_t  fw_major;
    uint8_t  fw_minor;
    
    // Dati dell'ultimo heartbeat
    int8_t   last_rssi;
    uint8_t  free_heap_pct;
    
    // Flag per sapere se lo slot dell'array è occupato
    bool     is_allocated;      
};

class SlaveRegistry {
private:
    SlaveNode _nodes[MAX_SLAVES]; // Array statico per evitare frammentazione dell'heap

public:
    SlaveRegistry();

    // Registra un nuovo nodo o aggiorna uno esistente dopo un MSG_ANNOUNCE
    bool addSlave(uint8_t id, const PayloadAnnounce* announce);

    // Aggiorna lo stato di salute di un nodo dopo un MSG_HEARTBEAT
    bool updateHeartbeat(uint8_t id, const PayloadHeartbeat* hb);

    // Cerca un nodo per ID. Ritorna nullptr se non esiste.
    SlaveNode* getSlaveById(uint8_t id);

    // Controlla tutti i nodi e marca OFFLINE quelli il cui last_seen ha superato il timeout
    void checkTimeouts();

    // Ritorna il numero totale di nodi attualmente registrati (online e offline)
    uint8_t getRegisteredCount();
};
