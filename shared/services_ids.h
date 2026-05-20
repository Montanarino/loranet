#pragma once
#include <stdint.h>
 
// ============================================================
//  service_ids.h — catalogo ID servizi e comandi LMP v1.0
//  Condiviso tra master e slave. Non modificare senza
//  aggiornare PROTOCOL.md e l'altra parte del firmware.
// ============================================================
 
// --- ID Servizi ---
enum ServiceId : uint8_t {
    SVC_CORE        = 0x00,
    SVC_SENSOR      = 0x01,
    SVC_HTTP_SERVER = 0x02,
    SVC_RELAY       = 0x03,
    SVC_LOGGER      = 0x04,
    // 0x05–0x7F riservati LMP
    SVC_USER_BASE   = 0x80,  // servizi custom dell'utente da qui in poi
};
 
// --- Comandi CORE (0x00) ---
enum CmdCore : uint8_t {
    CORE_PING         = 0x01,  // → uptime_ms (4B)
    CORE_REBOOT       = 0x02,  // → ACK poi reboot
    CORE_GET_INFO     = 0x03,  // → PayloadAnnounce
    CORE_GET_SERVICES = 0x04,  // → PayloadServiceList
    CORE_SET_RELAY    = 0x05,  // args: uint8_t enable → ACK
    CORE_GET_HEAP     = 0x06,  // → free(4B) + total(4B)
};
 
// --- Comandi SENSOR (0x01) ---
enum CmdSensor : uint8_t {
    SENSOR_READ_ALL     = 0x01,  // → JSON compatto {t:22.5,h:60,p:1013}
    SENSOR_READ_FIELD   = 0x02,  // args: uint8_t field_id → float (4B)
    SENSOR_SET_INTERVAL = 0x03,  // args: uint32_t ms → ACK
    SENSOR_SET_PIN      = 0x04,  // args: uint8_t pin → ACK
    SENSOR_GET_STATS    = 0x05,  // → min/max/avg ultimi 10 campioni
};
 
// Field ID per SENSOR_READ_FIELD
enum SensorField : uint8_t {
    FIELD_TEMPERATURE = 0x01,
    FIELD_HUMIDITY    = 0x02,
    FIELD_PRESSURE    = 0x03,
    FIELD_CO2         = 0x04,
    FIELD_LUX         = 0x05,
    FIELD_ADC_RAW     = 0x06,
};
 
// --- Comandi HTTP_SERVER (0x02) ---
enum CmdHttp : uint8_t {
    HTTP_START      = 0x01,  // args: uint16_t port → ACK
    HTTP_STOP       = 0x02,  // → ACK
    HTTP_GET_STATUS = 0x03,  // → JSON {running,port,clients}
    HTTP_SET_SSID   = 0x04,  // args: char ssid[32] → ACK
    HTTP_SET_PASS   = 0x05,  // args: char pass[32] → ACK
};
 
// --- Comandi RELAY (0x03) ---
enum CmdRelay : uint8_t {
    RELAY_ENABLE        = 0x01,  // → ACK
    RELAY_DISABLE       = 0x02,  // → ACK
    RELAY_GET_STATS     = 0x03,  // → forwarded(4B) + dropped(4B)
    RELAY_SET_HOP_LIMIT = 0x04,  // args: uint8_t max_hops → ACK
};
 
// --- Config keys (usati con MSG_CONFIG) ---
// Usare come stringa nel campo ConfigParam.key
#define CFG_PIN          "pin"
#define CFG_INTERVAL_MS  "interval_ms"
#define CFG_SSID         "ssid"
#define CFG_PASS         "pass"
#define CFG_PORT         "port"
#define CFG_HOP_LIMIT    "hop_limit"
#define CFG_NAME         "name"
