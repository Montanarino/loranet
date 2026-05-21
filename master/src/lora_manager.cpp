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
// processRxByte(uint8_t incoming_byte, LmpFrame *out_frame)
// RIMANE IDENTICO A QUELLO CHE HAI GIÀ SCRITTO!
// Copialo/Incollalo qui senza modifiche.
// ==============================================================
bool LoRaManager::processRxByte(uint8_t incoming_byte, LmpFrame *out_frame) {
    // ... [Inserisci la tua logica switch(rx_state) qui] ...
    return false;
}