#pragma once
#include <Arduino.h>
#include <lora_protocol.h>
#include "config.h" 

// Stati della macchina a stati per il parser in ricezione
enum class ParserState {
    WAIT_MAGIC,   // In attesa del byte di sincronizzazione (0xAB)
    READ_HEADER,  // Lettura dell'header (restanti 7 byte)
    READ_PAYLOAD, // Lettura del payload (lunghezza variabile)
    READ_CRC      // Lettura e validazione del CRC finale
};

class LoRaManager {
private:
    ParserState _rx_state;
    LmpFrame    _rx_frame;      // Buffer temporaneo in ricezione
    uint16_t    _rx_index;      // Cursore di lettura
    uint32_t    _err_count;     // Contatore errori (CRC fallito o frame malformati)
    uint16_t    _tx_seq;        // Contatore sequenziale per i frame in uscita

    // Metodo interno che elabora il flusso byte per byte
    bool processRxByte(uint8_t incoming_byte, LmpFrame *out_frame);

public:
    LoRaManager();
    
    // Inizializza l'hardware radio (UART o SPI a seconda del config.h)
    bool begin(); 
    
    // Interroga la radio. Restituisce true se ha composto un frame valido.
    bool poll(LmpFrame *out_frame); 
    
    // Invia byte grezzi alla radio
    void sendRaw(const uint8_t *data, size_t len);
    void sendFrame(LmpFrame *frame);
    
    // Genera il prossimo numero di sequenza per le trasmissioni
    uint16_t getNextSeq(); 
    
    // Restituisce il numero di pacchetti corrotti ricevuti
    uint32_t getErrorCount() const { return _err_count; }
};