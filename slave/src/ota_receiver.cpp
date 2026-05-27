#include "ota_receiver.h"

OtaReceiver::OtaReceiver() {
    _is_ongoing = false;
    _next_chunk_idx = 0;
}

void OtaReceiver::handleStart(const PayloadOtaStart* start, PayloadOtaAck* out_ack) {
    if (_is_ongoing) {
        out_ack->status = ACK_BUSY;
        strncpy(out_ack->message, "OTA IN PROG", 16);
        return;
    }

    _type = (OtaType)start->ota_type;
    _total_size = start->total_size;
    _total_chunks = start->total_chunks;
    _crc32_full = start->crc32_full;
    _next_chunk_idx = 0;
    _received_size = 0;

    if (_type == OTA_TYPE_FIRMWARE) {
        if (!Update.begin(_total_size)) {
            out_ack->status = ACK_ERROR;
            strncpy(out_ack->message, "UPDATE BEGIN ERR", 16);
            return;
        }
    }

    _is_ongoing = true;
    out_ack->status = ACK_OTA_READY;
    out_ack->chunk_index = 0xFFFF;
    strncpy(out_ack->message, "READY", 16);
    Serial.printf("[OTA] Inizio ricezione OTA: %d byte, %d chunk\n", _total_size, _total_chunks);
}

void OtaReceiver::handleChunk(const PayloadOtaChunk* chunk, PayloadOtaAck* out_ack) {
    if (!_is_ongoing) {
        out_ack->status = ACK_ERROR;
        return;
    }

    if (chunk->chunk_index != _next_chunk_idx) {
        out_ack->status = ACK_OTA_CHUNK_ERR;
        out_ack->chunk_index = _next_chunk_idx; // Comunica quale ci aspettavamo
        strncpy(out_ack->message, "WRONG INDEX", 16);
        return;
    }

    if (_type == OTA_TYPE_FIRMWARE) {
        if (Update.write((uint8_t*)chunk->data, chunk->data_len) != chunk->data_len) {
            out_ack->status = ACK_ERROR;
            strncpy(out_ack->message, "WRITE ERR", 16);
            _is_ongoing = false;
            Update.abort();
            return;
        }
    } else {
        // Per ora logghiamo solo i dati se non è firmware
        Serial.printf("[OTA] Ricevuti %d byte di config\n", chunk->data_len);
    }

    _received_size += chunk->data_len;
    _next_chunk_idx++;

    out_ack->status = ACK_OTA_CHUNK_OK;
    out_ack->chunk_index = chunk->chunk_index;
    strncpy(out_ack->message, "OK", 16);
    Serial.printf("[OTA] Ricevuto chunk %d/%d\n", _next_chunk_idx, _total_chunks);
}

void OtaReceiver::handleEnd(const PayloadOtaEnd* end, PayloadOtaAck* out_ack) {
    if (!_is_ongoing) {
        out_ack->status = ACK_ERROR;
        return;
    }

    // Qui dovremmo verificare il CRC32 finale se lo abbiamo calcolato
    // Per ora ci fidiamo di Update.end() per il firmware
    
    if (_type == OTA_TYPE_FIRMWARE) {
        if (Update.end(true)) {
            out_ack->status = ACK_OTA_DONE;
            strncpy(out_ack->message, "REBOOTING", 16);
            Serial.println("[OTA] Firmware ricevuto con successo. Riavvio...");
            _is_ongoing = false;
            delay(1000);
            ESP.restart();
        } else {
            out_ack->status = ACK_OTA_FAIL;
            Serial.printf("[OTA] Errore finale: %s\n", Update.errorString());
            strncpy(out_ack->message, "UPDATE END ERR", 16);
            _is_ongoing = false;
        }
    } else {
        out_ack->status = ACK_OTA_DONE;
        strncpy(out_ack->message, "CONFIG DONE", 16);
        _is_ongoing = false;
    }
}

void OtaReceiver::reset() {
    if (_is_ongoing && _type == OTA_TYPE_FIRMWARE) Update.abort();
    _is_ongoing = false;
    _next_chunk_idx = 0;
}
