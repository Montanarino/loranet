#pragma once
#include <Arduino.h>
#include "shared/lora_protocol.h"

// Stati possibili del parser
enum class ParserState {
    WAIT_MAGIC,   // In attesa del byte magico 0xAB
    READ_HEADER,  // Lettura dell'header (7 byte rimanenti)
    READ_PAYLOAD, // Lettura del payload (lunghezza variabile definita nell'header)
    READ_CRC      // Lettura dei 2 byte del CRC
};

class LoRaManager {
private:
    ParserState _rx_state;
    LmpFrame _rx_frame;      // Buffer temporaneo per il frame in arrivo
    uint16_t _rx_index;      // Indice del byte corrente in lettura
    uint32_t _err_count;     // Contatore dei pacchetti scartati (per statistiche)

    // Metodo interno che gestisce la logica byte per byte
    bool processRxByte(uint8_t incoming_byte, LmpFrame *out_frame);

public:
    LoRaManager();
    void begin(); // Inizializza la Seriale e il modulo LoRa
    
    // Da chiamare ripetutamente nel loop() o nel task FreeRTOS
    // Restituisce true se un pacchetto completo e valido è pronto
    bool poll(LmpFrame *out_frame); 
};