#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================
// 1. Definizioni di Base e Indirizzamento
// ==========================================
#define LMP_MAGIC_BYTE      0xAB  // Byte di sincronizzazione stream
#define LMP_ADDR_MASTER     0x00  // Indirizzo riservato al Master
#define LMP_ADDR_BROADCAST  0xFF  // Indirizzo di Broadcast

#define LMP_MAX_PAYLOAD_LEN 180   // Lunghezza massima del payload
#define LMP_MAX_FRAME_LEN   190   // Lunghezza massima del frame (10B overhead + 180B payload)
#define LMP_FRAME_OVERHEAD  10    // Overhead fisso del frame (8B header + 2B CRC)

#define LMP_CRC_INIT        0xFFFF// Valore iniziale del CRC-16
#define LMP_CRC_POLY        0x1021// Polinomio CRC-16/CCITT-FALSE

// ==========================================
// 2. Parametri di Timing (Valori di Default)
// ==========================================
#define ACK_TIMEOUT_MS         2000  // Attesa ACK prima del retry
#define ACK_RETRIES            3     // Tentativi massimi prima di rinunciare
#define CMD_TIMEOUT_MS         5000  // Attesa del CMD_RESULT da parte del Master
#define DISCOVERY_INTERVAL_MS  10000 // Frequenza DISCOVERY_REQ del Master
#define HEARTBEAT_INTERVAL_MS  10000 // Frequenza HEARTBEAT dello Slave
#define HEARTBEAT_TIMEOUT_MS   30000 // Timeout dopo il quale uno Slave è considerato OFFLINE
#define OTA_CHUNK_DELAY_MS     500   // Pausa tra l'invio di chunk OTA consecutivi
#define ANNOUNCE_JITTER_MAX    500   // Jitter massimo per la risposta al Discovery
#define STANDALONE_RETRY_MS    5000  // Frequenza di retry ANNOUNCE in modalità standalone

// ==========================================
// 3. Enumeratori del Protocollo
// ==========================================

// Tipi di Messaggio (MsgType)
typedef enum {
    MSG_ANNOUNCE      = 0x01, // Slave -> Master (Boot / Discovery) [ACK richiesto]
    MSG_ACK           = 0x02, // Bidirezionale (Conferma ricezione)
    MSG_DISCOVERY_REQ = 0x03, // Master -> Broadcast (Ricerca nodi)
    MSG_SERVICE_LIST  = 0x04, // Slave -> Master (Lista servizi completi)
    MSG_CMD           = 0x05, // Master -> Slave (Esecuzione comando) [ACK richiesto]
    MSG_CMD_RESULT    = 0x06, // Slave -> Master (Risultato comando)
    MSG_OTA_START     = 0x10, // Master -> Slave (Inizio OTA) [ACK richiesto]
    MSG_OTA_CHUNK     = 0x11, // Master -> Slave (Frazione firmware) [ACK richiesto]
    MSG_OTA_END       = 0x12, // Master -> Slave (Fine OTA) [ACK richiesto]
    MSG_OTA_ACK       = 0x13, // Slave -> Master (Stato specifico OTA)
    MSG_RELAY_FWD     = 0x20, // Relay -> Qualsiasi (Inoltro pacchetto mesh)
    MSG_HEARTBEAT     = 0x30, // Slave -> Master (Keep-alive periodico)
    MSG_CONFIG        = 0x40, // Master -> Slave (Configurazione parametri) [ACK richiesto]
    MSG_CONFIG_ACK    = 0x41  // Slave -> Master (Conferma applicazione config)
} MsgType;

// Codici di Stato (AckStatus)
typedef enum {
    ACK_OK            = 0x00, // Operazione completata con successo
    ACK_ERROR         = 0x01, // Errore generico
    ACK_BUSY          = 0x02, // Nodo occupato (es. OTA già in corso)
    ACK_UNKNOWN_CMD   = 0x03, // Comando o Servizio non riconosciuto
    ACK_NOT_FOUND     = 0x04, // Risorsa richiesta non trovata
    ACK_OTA_READY     = 0x10, // Slave pronto a ricevere i chunk OTA
    ACK_OTA_APPLYING  = 0x11, // Slave in scrittura finale / validazione
    ACK_OTA_CHUNK_OK  = 0x12, // Chunk ricevuto e scritto correttamente
    ACK_OTA_CHUNK_ERR = 0x13, // Chunk corrotto (fallimento CRC locale)
    ACK_OTA_DONE      = 0x14, // Intero OTA ricevuto con successo, riavvio imminente
    ACK_OTA_FAIL      = 0x15  // OTA fallito (mismatch CRC32 finale)
} AckStatus;

// ID Servizi di Sistema Riservati
typedef enum {
    SVC_CORE          = 0x00, // Comandi di sistema e diagnostica basica
    SVC_SENSOR        = 0x01, // Gestione sensori (DHT22, ADC, ecc.)
    SVC_HTTP_SERVER   = 0x02, // Configurazione/Stato Web Server locale
    SVC_RELAY         = 0x03, // Configurazione e statistiche repeater mesh
    SVC_LOGGER        = 0x04  // Servizio di log e memorizzazione
} ServiceId;

// Tipi di Trasferimento OTA
typedef enum {
    OTA_TYPE_FIRMWARE   = 0x01, // Binario completo dell'applicazione ESP32
    OTA_TYPE_CONFIG     = 0x02, // Blob o stringhe di configurazione destinate alla NVS
    OTA_TYPE_MODULE_SEL = 0x03  // Selezione/Attivazione dinamica moduli precompilati
} OtaType;

// ==========================================
// 4. Strutture Payload (Strictly Packed)
// ==========================================
#pragma pack(push, 1)

// Payload: ANNOUNCE (0x01)
typedef struct {
    char    name[16];         // Nome leggibile del nodo (null-terminated)
    uint8_t service_count;    // Numero di servizi offerti (0-8)
    uint8_t fw_major;         // Versione maggiore del firmware
    uint8_t fw_minor;         // Versione minore del firmware
    uint8_t flags;            // bit 0: capace di fare da relay | bit 1-7: riservati
    uint8_t hop;              // 0 = diretto al master, 1+ = passato via relay
    uint8_t relay_src;        // ID del nodo che ha fatto da ultimo relay (valido se hop > 0)
} PayloadAnnounce;

// Payload: ACK (0x02)
typedef struct {
    uint16_t ack_seq;         // SEQ del messaggio che si sta confermando
    uint8_t  status;          // Codice di stato (AckStatus)
    char     message[8];      // Messaggio testuale opzionale (null-terminated)
} PayloadAck;

// Entry singola per la lista dei servizi
typedef struct {
    uint8_t service_id;       // ID univoco del servizio (ServiceId o Custom)
    char    name[16];         // Nome identificativo (es. "dht22_sensor")
    uint8_t version;          // Versione dell'implementazione del servizio
    uint8_t active;           // 1 = attivo ed in esecuzione, 0 = disattivato
} ServiceEntry;

// Payload: SERVICE_LIST (0x04)
typedef struct {
    uint8_t      count;       // Numero effettivo di servizi inclusi (max 8)
    ServiceEntry services[8]; // Array statico dei descrittori di servizio
} PayloadServiceList;

// Payload: CMD (0x05)
typedef struct {
    uint8_t service_id;       // ID del servizio destinatario del comando
    uint8_t cmd_id;           // ID del comando specifico del servizio
    uint8_t args_len;         // Lunghezza in byte degli argomenti raw
    uint8_t args[64];         // Argomenti del comando (interpretazione custom)
} PayloadCmd;

// Payload: CMD_RESULT (0x06)
typedef struct {
    uint8_t service_id;       // ID del servizio che ha eseguito il comando
    uint8_t cmd_id;           // ID del comando associato al risultato
    uint8_t status;           // 0x00 = successo, altri valori = errore del servizio
    uint8_t data_len;         // Lunghezza in byte dei dati di risposta
    uint8_t data[64];         // Dati risultanti restituiti al Master
} PayloadCmdResult;

// Payload: OTA_START (0x10)
typedef struct {
    uint8_t  ota_type;        // Tipologia di OTA (OtaType)
    uint32_t total_size;      // Dimensione totale del file/blob in byte
    uint16_t chunk_size;      // Dimensione di ogni singolo pacchetto chunk (max 128)
    uint16_t total_chunks;    // Numero totale di chunk attesi (ceil(total_size/chunk_size))
    uint32_t crc32_full;      // CRC32 dell'intero file per la convalida finale dello Slave
    char     version[8];      // Stringa della versione target (es. "1.2.0")
} PayloadOtaStart;

// Payload: OTA_CHUNK (0x11)
typedef struct {
    uint16_t chunk_index;     // Indice progressivo del chunk (0-based)
    uint8_t  data_len;        // Byte validi contenuti nel buffer data
    uint8_t  data[128];       // Dati grezzi del firmware/configurazione
} PayloadOtaChunk;

// Payload: OTA_END (0x12)
typedef struct {
    uint32_t final_crc32;     // Riconferma del CRC32 dell'intero payload per sicurezza
} PayloadOtaEnd;

// Payload: OTA_ACK (0x13)
typedef struct {
    uint8_t  status;          // Stato dell'operazione OTA (AckStatus)
    uint16_t chunk_index;     // Indice del chunk a cui si riferisce (0xFFFF se globale)
    char     message[16];     // Messaggio descrittivo opzionale (es. "crc mismatch")
} PayloadOtaAck;

// Payload: HEARTBEAT (0x30)
typedef struct {
    uint32_t uptime_s;        // Secondi trascorsi dal boot dello Slave
    int8_t   last_rssi;       // RSSI dell'ultimo frame valido ricevuto (in dBm)
    uint8_t  free_heap_pct;   // Percentuale di heap libero: (free/total) * 100
    uint8_t  active_services; // Bitmask dei servizi attivi (Bit N = Servizio N attivo)
    uint8_t  relay_mode;      // 1 se la modalità ripetitore mesh è attualmente attiva, 0 se disattivata
} PayloadHeartbeat;

// Coppia chiave/valore per la configurazione
typedef struct {
    char key[12];             // Nome del parametro (es. "pin_dht", "ssid")
    char value[20];           // Valore espresso come stringa (es. "22", "NetWork")
} ConfigParam;

// Payload: CONFIG (0x40)
typedef struct {
    uint8_t     service_id;   // Servizio target a cui applicare la configurazione
    uint8_t     param_count;  // Numero effettivo di parametri inclusi (max 4)
    ConfigParam params[4];    // Array di coppie chiave/valore
} PayloadConfig;

// Payload: CONFIG_ACK (0x41)
typedef struct {
    uint8_t service_id;       // ID del servizio configurato
    uint8_t status;           // 0x00 = OK, 0x01 = Svc non trovato, 0x02 = Param ignoto, 0x03 = Valore errato
    uint8_t applied;          // Numero di parametri effettivamente validati e scritti in NVS
} PayloadConfigAck;

// ==========================================
// 5. Struttura Frame Principale
// ==========================================

// Header fisso del Frame (8 byte)
typedef struct {
    uint8_t  magic;           // Deve essere sempre LMP_MAGIC_BYTE (0xAB)
    uint8_t  dst;             // Indirizzo del destinatario
    uint8_t  src;             // Indirizzo del mittente
    uint8_t  type;            // Tipo di messaggio (MsgType castato a uint8_t)
    uint16_t seq;             // Numero di sequenza (Little Endian, 0xFFFF = Non applicabile)
    uint16_t len;             // Lunghezza in byte del payload effettivo (0 - 180)
} LmpFrameHeader;

// Struttura Frame Completa (Dimensione di memoria massima)
typedef struct {
    LmpFrameHeader header;
    uint8_t        payload[LMP_MAX_PAYLOAD_LEN]; // Buffer del payload a capacità massima
    uint16_t       crc16;                        // CRC-16 posizionato alla fine della memoria statica
} LmpFrame;

#pragma pack(pop)

// ==========================================
// 6. Funzioni Condivise (Encode / Decode / CRC)
// ==========================================

/**
 * @brief Calcola il CRC-16/CCITT-FALSE sull'array di byte specificato.
 * Algoritmo: Polinomio 0x1021, Valore Iniziale 0xFFFF, Nessun XOR finale, Nessun reverse.
 */
uint16_t lmp_calculate_crc(const uint8_t *data, uint16_t len);

/**
 * @brief Confeziona un payload in un frame LMP strutturato pronto per la trasmissione (Encode).
 * Valorizza in automatico il byte MAGIC, la lunghezza e calcola il CRC posizionandolo
 * immediatamente dopo l'ultimo byte del payload per ottimizzare la trasmissione su UART.
 * * @return La dimensione effettiva del pacchetto da trasmettere sulla rete (8 + payload_len + 2).
 * Restituisce 0 in caso di parametri non validi.
 */
uint16_t lmp_build_frame(LmpFrame *out_frame, uint8_t dst, uint8_t src, MsgType type, uint16_t seq, const void *payload, uint16_t payload_len);

/**
 * @brief Convalida l'integrità e la correttezza sintattica di un frame LMP ricevuto (Decode).
 * Controlla il byte MAGIC, i limiti di lunghezza del payload ed esegue il calcolo locale del CRC
 * confrontandolo con quello presente nel pacchetto.
 * * @return true se il frame è integro e conforme alle specifiche LMP, false altrimenti.
 */
bool lmp_validate_frame(const LmpFrame *frame);

#ifdef __cplusplus
}
#endif

#endif // LORA_PROTOCOL_H