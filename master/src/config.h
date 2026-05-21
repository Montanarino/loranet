#pragma once

// =========================================================================
// CONFIGURAZIONE IDENTITÀ NODO
// =========================================================================
#define SLAVE_ID       0x01             // Indirizzo univoco del nodo (0x01 - 0xFE)
#define SLAVE_NAME     "Sensore_Serra_1" // Identificativo letterale (max 16 caratteri)
#define CAPABLE_RELAY  true             // true se la scheda supporta la modalità ripetitore

// =========================================================================
// INTERFACCIA HARDWARE
// =========================================================================
// Sulla LilyGO T3 commentiamo l'UART e attiviamo l'SPI
// #define LORA_INTERFACE_UART
#define LORA_INTERFACE_SPI

// =========================================================================
// CONFIGURAZIONE PIN SPI (Specifici per LilyGO T3 v1.6.1)
// =========================================================================
#ifdef LORA_INTERFACE_SPI
    // Pin del bus SPI mappati sulla T3 v1.6.1
    #define LORA_SPI_SCK      5
    #define LORA_SPI_MISO     19
    #define LORA_SPI_MOSI     27
    
    // Pin di controllo del chip LoRa
    #define LORA_SPI_CS       18
    #define LORA_SPI_RST      23   // Nota: sulle vecchie versioni V1 era il 14, sulla v1.6.1 è il 23
    #define LORA_SPI_DIO0     26   // Pin di interrupt fondamentale per la ricezione
    
    // --- PARAMETRI RADIO (MOLTO IMPORTANTE) ---
    // Controlla l'etichetta o il chip antenna dietro la tua LilyGO!
    // Se c'è scritto 433, usa 433E6. Se c'è scritto 868, usa 868E6.
    #define LORA_FREQ         868E6     // Frequenza operativa (es. 868 MHz per l'Europa)
    
    #define LORA_SF           9         // Spreading Factor (da specifiche del tuo protocollo)
    #define LORA_BW           125E3     // Bandwidth in Hz
#endif