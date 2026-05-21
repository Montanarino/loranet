#pragma once

#include <Arduino.h>
#include <lora_protocol.h>

// Numero massimo di nodi supportati in memoria.
// Puoi alzarlo o abbassarlo in base a quanti dispositivi fisici prevedi di avere.
#define MAX_SLAVES 32 

// Struttura che rappresenta lo stato di un singolo nodo nella rete
struct SlaveNode {
    uint8_t  id;                // Indirizzo LoRa dello slave (0x01 - 0xFE)
    char     name[17];          // Nome letterale (16 char + terminatore '\0')
    uint32_t last_seen;         // Timestamp (millis) dell'ultimo contatto radio
    bool     is_online;         // true se il nodo sta rispondendo regolarmente
    bool     is_repeater;       // true se il nodo ha la capacità di fare da relay mesh
    uint8_t  hop_count;         // 0 = diretto, >0 = inoltrato da un relay
    uint8_t  relay_src;         // ID del nodo che ha eventualmente inoltrato il pacchetto
    
    // Info di sistema
    uint8_t  fw_major;
    uint8_t  fw_minor;
    
    // Dati diagnostici (aggiornati quando implementerai il messaggio Heartbeat)
    int8_t   last_rssi;         // Potenza del segnale
    uint8_t  free_heap_pct;     // Memoria libera sul nodo
    
    // Stato di allocazione in memoria
    bool     is_allocated;      // true se questo slot dell'array è occupato da un nodo
};

class SlaveRegistry {
private:
    SlaveNode _nodes[MAX_SLAVES]; // Array statico pre-allocato

public:
    SlaveRegistry();

    // Registra un nuovo nodo o aggiorna uno esistente alla ricezione di MSG_ANNOUNCE
    bool addSlave(uint8_t id, const PayloadAnnounce* announce);

    // Aggiorna lo stato di salute e il timeout di un nodo alla ricezione di MSG_HEARTBEAT
    bool updateHeartbeat(uint8_t id, const PayloadHeartbeat* hb);

    // Cerca un nodo specifico per ID. Ritorna nullptr se il nodo non è in rubrica.
    SlaveNode* getSlaveById(uint8_t id);

    // Funzione di pulizia: marca come OFFLINE i nodi che non si fanno sentire da troppo tempo
    void checkTimeouts();

    // Ritorna il numero totale di nodi registrati nel sistema (sia online che offline)
    uint8_t getRegisteredCount() const;
};