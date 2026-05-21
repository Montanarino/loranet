#pragma once
#include <Arduino.h>
#include "lora_protocol.h"
#include "config.h" // Includiamo i pin e la scelta dell'interfaccia

// Stati possibili del parser
enum class ParserState {
    WAIT_MAGIC,
    READ_HEADER,
    READ_PAYLOAD,
    READ_CRC
};

class LoRaManager {
private:
    ParserState _rx_state;
    LmpFrame _rx_frame;      
    uint16_t _rx_index;      
    uint32_t _err_count;     
    uint16_t _tx_seq;        // Contatore di sequenza per l'invio (Da protocollo)

    bool processRxByte(uint8_t incoming_byte, LmpFrame *out_frame);

public:
    LoRaManager();
    
    bool begin(); // Ora restituisce bool per confermare l'inizializzazione
    bool poll(LmpFrame *out_frame); 
    
    // Funzioni aggiunte per astrazione e protocollo
    void sendRaw(const uint8_t *data, size_t len);
    uint16_t getNextSeq(); 
};