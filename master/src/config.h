#pragma once

// =========================================================================
// 1. CONFIGURAZIONE IDENTITÀ COORDINATORE (MASTER)
// =========================================================================
// Il Master ha l'indirizzo fisso 0x00 definito rigidamente dal protocollo.
// Non necessita di ID o nomi personalizzati per l'auto-annuncio.

// =========================================================================
// 2. SCELTA DELL'INTERFACCIA HARDWARE (LilyGO T3 v1.6.1)
// =========================================================================
// Anche il Master usa il chip LoRa integrato su bus SPI, quindi ignoriamo l'UART
// #define LORA_INTERFACE_UART
#define LORA_INTERFACE_SPI

// =========================================================================
// 3. CONFIGURAZIONE PIN SPI E RADIO
// =========================================================================
#ifdef LORA_INTERFACE_SPI
    // Pin del bus SPI specifici della T3 v1.6.1
    #define LORA_SPI_SCK      5
    #define LORA_SPI_MISO     19
    #define LORA_SPI_MOSI     27
    
    // Pin di controllo del chip SX1276/1278
    #define LORA_SPI_CS       18
    #define LORA_SPI_RST      23   // Pin di reset per la versione v1.6.1
    #define LORA_SPI_DIO0     26   // Pin di interrupt (fondamentale per i task FreeRTOS)
    
    // --- PARAMETRI DI RETE LORA ---
    // ATTENZIONE: Questo parametro deve essere IDENTICO a quello dello Slave, 
    // altrimenti le due schede non si sentiranno in radiofrequenza.
    #define LORA_FREQ         868E6     // Frequenza operativa (es. 868 MHz)
    #define LORA_SF           9         // Spreading Factor (LMP v1.0)
    #define LORA_BW           125E3     // Bandwidth in Hz
#endif