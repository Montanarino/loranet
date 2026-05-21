#include "lora_manager.h"

// Inclusione condizionale della libreria SPI/LoRa se richiesto da config.h
#ifdef LORA_INTERFACE_SPI
#include <SPI.h>
#include <LoRa.h> 
#endif

LoRaManager::LoRaManager() {
    _rx_state = ParserState::WAIT_MAGIC;
    _rx_index = 0;
    _err_count = 0;
    _tx_seq = 1; // Come da protocollo, il SEQ parte da 1
}

bool LoRaManager::begin() {
#ifdef LORA_INTERFACE_UART
    // Inizializza Seriale 2 per i moduli trasparenti (es. Ebyte)
    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_UART_RX_PIN, LORA_UART_TX_PIN);
    return true; 
    
#elif defined(LORA_INTERFACE_SPI)
    // Inizializza i pin SPI per i chip LoRa "nudi" (es. SX1278)
    SPI.begin(LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI, LORA_SPI_CS);
    LoRa.setSPI(SPI);
    LoRa.setPins(LORA_SPI_CS, LORA_SPI_RST, LORA_SPI_DIO0);
    
    if (!LoRa.begin(LORA_FREQ)) {
        return false; 
    }
    
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    return true;
#else
    #error "Nessuna interfaccia LoRa definita in config.h!"
    return false;
#endif
}

bool LoRaManager::poll(LmpFrame *out_frame) {
#ifdef LORA_INTERFACE_UART
    while (Serial2.available() > 0) {
        if (processRxByte(Serial2.read(), out_frame)) {
            return true;
        }
    }
#elif defined(LORA_INTERFACE_SPI)
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        while (LoRa.available()) {
            if (processRxByte(LoRa.read(), out_frame)) {
                // Svuotiamo il resto del buffer se abbiamo trovato un frame valido
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
    LoRa.beginPacket();
    LoRa.write(data, len);
    LoRa.endPacket();
#endif
}

void LoRaManager::sendFrame(LmpFrame* frame) {
    uint8_t wireBuffer[256];
    uint16_t len = frame->header.len;
    
    // 1. Copia Header
    memcpy(wireBuffer, &frame->header, sizeof(LmpFrameHeader));
    // 2. Copia Payload
    if (len > 0) {
        memcpy(wireBuffer + sizeof(LmpFrameHeader), frame->payload, len);
    }
    // 3. Estrae il vero CRC usando il nome corretto (crc16)
    uint16_t crc_idx = sizeof(LmpFrameHeader) + len;
    wireBuffer[crc_idx] = frame->crc16 & 0xFF;
    wireBuffer[crc_idx + 1] = (frame->crc16 >> 8) & 0xFF;
    
    // Invia i byte compattati
    sendRaw(wireBuffer, crc_idx + 2);
}

uint16_t LoRaManager::getNextSeq() {
    uint16_t seq = _tx_seq++;
    if (_tx_seq == 0) {
        _tx_seq = 1; // Wrappa a 1, non a 0
    }
    return seq;
}

bool LoRaManager::processRxByte(uint8_t incoming_byte, LmpFrame *out_frame) {
    uint8_t* frame_buffer = (uint8_t*)&_rx_frame;

    switch (_rx_state) {
        case ParserState::WAIT_MAGIC:
            if (incoming_byte == LMP_MAGIC_BYTE) {
                frame_buffer[0] = incoming_byte;
                _rx_index = 1;
                _rx_state = ParserState::READ_HEADER;
            }
            break;

        case ParserState::READ_HEADER:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // Header completo (8 byte)
            if (_rx_index == sizeof(LmpFrameHeader)) {
                if (_rx_frame.header.len > LMP_MAX_PAYLOAD_LEN) {
                    _err_count++;
                    _rx_state = ParserState::WAIT_MAGIC; // Lunghezza non valida, reset
                } else if (_rx_frame.header.len == 0) {
                    _rx_state = ParserState::READ_CRC;   // Frame senza payload (es. PING)
                } else {
                    _rx_state = ParserState::READ_PAYLOAD;
                }
            }
            break;

        case ParserState::READ_PAYLOAD:
            frame_buffer[_rx_index++] = incoming_byte;
            
            // Payload completo
            if (_rx_index == sizeof(LmpFrameHeader) + _rx_frame.header.len) {
                _rx_state = ParserState::READ_CRC;
            }
            break;

        case ParserState::READ_CRC: {
            uint16_t expected_total_size = sizeof(LmpFrameHeader) + _rx_frame.header.len + 2;
            uint16_t crc_byte_index = _rx_index - (expected_total_size - 2); 
            
            if (crc_byte_index == 0) {
                _rx_frame.crc16 = incoming_byte; // Primo byte (LSB)
                _rx_index++;
            } else if (crc_byte_index == 1) {
                _rx_frame.crc16 |= (incoming_byte << 8); // Secondo byte (MSB)
                _rx_index++;
                
                // Valida il frame!
                if (lmp_validate_frame(&_rx_frame)) {
                    memcpy(out_frame, &_rx_frame, sizeof(LmpFrame));
                    _rx_state = ParserState::WAIT_MAGIC;
                    return true;
                } else {
                    Serial.println("[DEBUG] CRC Fallito! Dati corrotti in aria.");
                    _err_count++;
                    _rx_state = ParserState::WAIT_MAGIC;
                }
            }
            break;
        }
    }
    
    return false;
}