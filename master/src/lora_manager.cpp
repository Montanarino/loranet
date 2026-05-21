#include "lora_manager.h"

// Se è scelto l'SPI, includiamo la libreria standard LoRa
#ifdef LORA_INTERFACE_SPI
#include <SPI.h>
#include <LoRa.h> // Richiede la libreria sandeepmistry/LoRa
#endif

LoRaManager::LoRaManager() {
    _rx_state = ParserState::WAIT_MAGIC;
    _rx_index = 0;
    _err_count = 0;
    _tx_seq = 1; // Inizializza SEQ a 1 come da specifiche
}

bool LoRaManager::begin() {
#ifdef LORA_INTERFACE_UART
    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_UART_RX_PIN, LORA_UART_TX_PIN);
    return true; // L'UART non ha un vero check di fallimento su ESP32
    
#elif defined(LORA_INTERFACE_SPI)
    SPI.begin(LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI, LORA_SPI_CS);
    LoRa.setPins(LORA_SPI_CS, LORA_SPI_RST, LORA_SPI_DIO0);
    
    if (!LoRa.begin(LORA_FREQ)) {
        return false; // Errore di connessione SPI o chip rotto
    }
    
    // Configurazione PHY per SPI
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    // LoRa.setCodingRate4(5); // Opzionale se vuoi forzare il CR
    return true;
#endif
}

bool LoRaManager::poll(LmpFrame *out_frame) {
#ifdef LORA_INTERFACE_UART
    while (Serial2.available() > 0) {
        if (processRxByte(Serial2.read(), out_frame)) return true;
    }
    
#elif defined(LORA_INTERFACE_SPI)
    // I moduli SPI ricevono pacchetti interi, non stream continui
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        while (LoRa.available()) {
            if (processRxByte(LoRa.read(), out_frame)) {
                // Se il pacchetto è valido, svuotiamo eventuale spazzatura rimasta nel buffer radio
                while(LoRa.available()) LoRa.read();
                return true;
            }
        }
    }
#endif
    return false;
}

void LoRaManager::sendRaw(const uint8_t *data, size_t len) {
#ifdef LORA_INTERFACE_UART
    Serial2.write(data, len);
    
#elif defined(LORA_INTERFACE_SPI)
    // Invio di un pacchetto strutturato via SPI
    LoRa.beginPacket();
    LoRa.write(data, len);
    LoRa.endPacket();
#endif
}

uint16_t LoRaManager::getNextSeq() {
    uint16_t seq = _tx_seq++;
    // Il protocollo prevede che il contatore wrappi a 1 (0 = non applicabile)
    if (_tx_seq == 0) {
        _tx_seq = 1;
    }
    return seq;
}

// ==============================================================
// processRxByte: Macchina a stati per il parsing non bloccante 
//                del flusso di byte LoRa
// ==============================================================
bool LoRaManager::processRxByte(uint8_t incoming_byte, LmpFrame *out_frame) {
    // Puntatore usato per scrivere i byte direttamente nella memoria della struttura _rx_frame
    uint8_t* frame_buffer = (uint8_t*)&_rx_frame;

    switch (_rx_state) {
        case ParserState::WAIT_MAGIC:
            // 1. Attendiamo il byte di sincronizzazione (0xAB) per iniziare a registrare
            if (incoming_byte == LMP_MAGIC_BYTE) {
                frame_buffer[0] = incoming_byte;
                _rx_index = 1;
                _rx_state = ParserState::READ_HEADER;
            }
            break;

        case ParserState::READ_HEADER:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // 2. Quando abbiamo letto tutti gli 8 byte dell'header (sizeof(LmpFrameHeader))
            if (_rx_index == sizeof(LmpFrameHeader)) {
                // Controlliamo subito se la lunghezza del payload dichiarata è valida e sicura
                if (_rx_frame.header.len > LMP_MAX_PAYLOAD_LEN) {
                    _err_count++;
                    _rx_state = ParserState::WAIT_MAGIC; // Lunghezza non valida (frame corrotto), reset
                } else if (_rx_frame.header.len == 0) {
                    _rx_state = ParserState::READ_CRC;   // Frame senza payload (es. PING), passiamo subito al CRC
                } else {
                    _rx_state = ParserState::READ_PAYLOAD; // Dobbiamo leggere il payload
                }
            }
            break;

        case ParserState::READ_PAYLOAD:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // 3. Quando abbiamo letto un numero di byte pari a quello dichiarato nell'header
            if (_rx_index == sizeof(LmpFrameHeader) + _rx_frame.header.len) {
                _rx_state = ParserState::READ_CRC;
            }
            break;

        case ParserState::READ_CRC:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // Calcoliamo la dimensione totale che ci aspettiamo (Header + Payload + 2 byte CRC)
            uint16_t expected_total_size = sizeof(LmpFrameHeader) + _rx_frame.header.len + 2;
            
            // 4. Quando abbiamo letto anche gli ultimi 2 byte del CRC
            if (_rx_index == expected_total_size) {
                // Validiamo il pacchetto intero ricalcolando il CRC (questa funzione è nel protocollo condiviso)
                if (lmp_validate_frame(&_rx_frame)) {
                    // Frame integro! Lo copiamo nel puntatore di uscita e resettiamo la macchina
                    memcpy(out_frame, &_rx_frame, expected_total_size);
                    _rx_state = ParserState::WAIT_MAGIC;
                    return true; // Diciamo al programma chiamante che c'è un nuovo pacchetto pronto e perfetto!
                } else {
                    // Il CRC fallisce: i dati radio sono stati corrotti da interferenze o collisioni
                    _err_count++;
                    _rx_state = ParserState::WAIT_MAGIC; // Scartiamo tutto in modo sicuro e resettiamo
                }
            }
            break;
    }
    
    // Ritorna false se stiamo ancora componendo il frame o se era solo rumore di fondo
    return false;
}