#include "lora_protocol.h"
#include <string.h>

uint16_t lmp_calculate_crc(const uint8_t *data, uint16_t len) {
    if (!data || len == 0) {
        return 0xFFFF;
    }

    uint16_t crc = LMP_CRC_INIT; // 0xFFFF
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ LMP_CRC_POLY; // 0x1021
            } else {
                crc = (crc << 1);
            }
        }
    }
    
    return crc;
}

uint16_t lmp_build_frame(LmpFrame *out_frame, uint8_t dst, uint8_t src, MsgType type, uint16_t seq, const void *payload, uint16_t payload_len) {
    // Controllo di validità dei puntatori e dei limiti del protocollo
    if (!out_frame || payload_len > LMP_MAX_PAYLOAD_LEN) {
        return 0;
    }
    if (payload_len > 0 && !payload) {
        return 0;
    }

    // 1. Compilazione dei campi dell'Header fisse
    out_frame->header.magic = LMP_MAGIC_BYTE;
    out_frame->header.dst   = dst;
    out_frame->header.src   = src;
    out_frame->header.type  = (uint8_t)type;
    out_frame->header.seq   = seq;
    out_frame->header.len   = payload_len;

    // 2. Copia del payload (se presente) immediatamente dopo l'header
    if (payload_len > 0) {
        memcpy(out_frame->payload, payload, payload_len);
    }

    // 3. Calcolo del CRC-16 sul pacchetto dinamico (Header + Payload effettivo)
    uint16_t bytes_to_crc = sizeof(LmpFrameHeader) + payload_len; // 8 + payload_len
    uint16_t calculated_crc = lmp_calculate_crc((const uint8_t*)out_frame, bytes_to_crc);

    // 4. Posizionamento del CRC in formato compatto (Wire format)
    // Iniettiamo il CRC direttamente nell'array dei byte subito dopo la fine reale del payload.
    // In questo modo, trasmettendo i byte consecutivi, il CRC si trova all'offset corretto.
    uint8_t *wire_crc_ptr = (uint8_t*)out_frame + bytes_to_crc;
    wire_crc_ptr[0] = (uint8_t)(calculated_crc & 0xFF);         // Byte meno significativo (LE)
    wire_crc_ptr[1] = (uint8_t)((calculated_crc >> 8) & 0xFF);  // Byte più significativo (LE)

    // Per ridondanza e consistenza della memoria, compiliamo anche il campo statico finale struct
    out_frame->crc16 = calculated_crc;

    // Restituisce la dimensione esatta in byte occupata dal frame reale sulla rete radio
    return bytes_to_crc + 2; 
}

bool lmp_validate_frame(const LmpFrame *frame) {
    if (!frame) {
        return false;
    }

    // Verifica preliminare del byte magico di sincronizzazione stream
    if (frame->header.magic != LMP_MAGIC_BYTE) {
        return false;
    }

    // Verifica che la lunghezza dichiarata non sfori i limiti fisici del buffer
    if (frame->header.len > LMP_MAX_PAYLOAD_LEN) {
        return false;
    }

    // Calcolo del CRC locale sull'estensione dinamica ricevuta (Header + Payload effettivo)
    uint16_t bytes_to_crc = sizeof(LmpFrameHeader) + frame->header.len;
    uint16_t local_crc = lmp_calculate_crc((const uint8_t*)frame, bytes_to_crc);

    // Estrazione del CRC dal formato compatto di rete (Wire format) posizionato dopo il payload
    const uint8_t *wire_crc_ptr = (const uint8_t*)frame + bytes_to_crc;
    uint16_t wire_crc = (uint16_t)(wire_crc_ptr[0] | (wire_crc_ptr[1] << 8));

    // Il pacchetto è considerato valido solo se i checksum coincidono perfettamente
    return (local_crc == wire_crc);
}