#include "lora_manager.h"

LoRaManager::LoRaManager() {
    _rx_state = ParserState::WAIT_MAGIC;
    _rx_index = 0;
    _err_count = 0;
}

void LoRaManager::begin() {
    // Inizializza la seriale hardware connessa al LoRa (es. Serial2)
    Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

bool LoRaManager::poll(LmpFrame *out_frame) {
    // Leggi tutti i byte disponibili nel buffer UART
    // Sostituisci Serial2 con la tua interfaccia seriale
    while (Serial2.available() > 0) {
        uint8_t byte_in = Serial2.read();
        
        // Passa il byte alla state machine
        if (processRxByte(byte_in, out_frame)) {
            // Abbiamo un frame completo e valido!
            return true; 
        }
    }
    return false;
}

bool LoRaManager::processRxByte(uint8_t incoming_byte, LmpFrame *out_frame) {
    
    // Cast del buffer della struct per potervi accedere byte per byte
    uint8_t* frame_buffer = (uint8_t*)&_rx_frame;

    switch (_rx_state) {
        
        case ParserState::WAIT_MAGIC:
            if (incoming_byte == LMP_MAGIC_BYTE) {
                frame_buffer[0] = incoming_byte;
                _rx_index = 1;
                _rx_state = ParserState::READ_HEADER;
            }
            // Se non è il MAGIC_BYTE, lo scartiamo in silenzio
            break;

        case ParserState::READ_HEADER:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // L'header è di 8 byte. Se ne abbiamo letti 8, l'header è completo
            if (_rx_index == sizeof(LmpFrameHeader)) {
                
                // Verifichiamo se il campo 'len' (lunghezza payload) è sensato
                if (_rx_frame.header.len > LMP_MAX_PAYLOAD_LEN) {
                    // Lunghezza non valida! C'è stato un errore di trasmissione.
                    // Resettiamo il parser e ricominciamo a cercare il MAGIC
                    _err_count++;
                    _rx_state = ParserState::WAIT_MAGIC;
                    
                } else if (_rx_frame.header.len == 0) {
                    // Se non c'è payload (es. PING o ACK senza argomenti), saltiamo alla lettura del CRC
                    _rx_state = ParserState::READ_CRC;
                } else {
                    // Se c'è payload, andiamo a leggerlo
                    _rx_state = ParserState::READ_PAYLOAD;
                }
            }
            break;

        case ParserState::READ_PAYLOAD:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // Controlliamo se abbiamo letto tutti i byte dichiarati nel campo 'len'
            if (_rx_index == sizeof(LmpFrameHeader) + _rx_frame.header.len) {
                _rx_state = ParserState::READ_CRC;
            }
            break;

        case ParserState::READ_CRC:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // Il CRC è di 2 byte. 
            // Indice finale atteso = Header(8) + Payload(len) + CRC(2)
            uint16_t expected_total_size = sizeof(LmpFrameHeader) + _rx_frame.header.len + 2;
            
            if (_rx_index == expected_total_size) {
                // Abbiamo letto tutti i byte del pacchetto!
                // Ora convalidiamo il CRC usando la funzione condivisa
                if (lmp_validate_frame(&_rx_frame)) {
                    
                    // CRC OK! Copiamo il frame validato nel puntatore di uscita
                    memcpy(out_frame, &_rx_frame, expected_total_size);
                    
                    // Resettiamo lo stato per il prossimo frame
                    _rx_state = ParserState::WAIT_MAGIC;
                    return true;
                    
                } else {
                    // CRC fallito! Il pacchetto è corrotto.
                    // Lo scartiamo silenziosamente e ricominciamo (come da specifiche)
                    _err_count++;
                    _rx_state = ParserState::WAIT_MAGIC;
                }
            }
            break;
    }
    
    return false; // Pacchetto non ancora completo
}