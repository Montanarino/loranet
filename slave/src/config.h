#pragma once

// ==========================================
// 1. SCELTA DELL'INTERFACCIA HARDWARE
// ==========================================
// De-commenta SOLO l'interfaccia che stai utilizzando fisicamente
#define LORA_INTERFACE_UART
// #define LORA_INTERFACE_SPI

// ==========================================
// 2. CONFIGURAZIONE PIN UART (Moduli Trasparenti)
// ==========================================
#ifdef LORA_INTERFACE_UART
    #define LORA_UART_TX_PIN 17
    #define LORA_UART_RX_PIN 16
    #define LORA_UART_BAUD   9600
#endif

// ==========================================
// 3. CONFIGURAZIONE PIN SPI (Chip SX127x nudi)
// ==========================================
#ifdef LORA_INTERFACE_SPI
    #define LORA_SPI_SCK  18
    #define LORA_SPI_MISO 19
    #define LORA_SPI_MOSI 23
    #define LORA_SPI_CS   5
    #define LORA_SPI_RST  14
    #define LORA_SPI_DIO0 26
    
    // I moduli UART usano i comandi AT per questi parametri, 
    // ma i moduli SPI devono configurarli via software all'avvio.
    #define LORA_FREQ     433E6 // Frequenza (es. 433 MHz)
    #define LORA_SF       9     // Spreading Factor
    #define LORA_BW       125E3 // Bandwidth in Hz
#endif