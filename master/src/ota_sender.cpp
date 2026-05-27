#include "ota_sender.h"

extern LoRaManager loraManager;
extern SemaphoreHandle_t radioMutex;

OtaSender::OtaSender() {
    _state = OtaSenderState::IDLE;
    _target_id = 0;
    _current_chunk = 0;
    _retries = 0;
}

bool OtaSender::begin(uint8_t target_id, const char* filename, OtaType type) {
    if (_state != OtaSenderState::IDLE) return false;

    _file = LittleFS.open(filename, "r");
    if (!_file) {
        Serial.printf("[OTA] Errore: Impossibile aprire il file %s\n", filename);
        return false;
    }

    _target_id = target_id;
    _type = type;
    _total_size = _file.size();
    _total_chunks = (_total_size + 127) / 128; // Chunk fissi da 128 byte
    _current_chunk = 0;
    _retries = 0;
    
    Serial.printf("[OTA] Inizio invio file %s (%d byte, %d chunk) a 0x%02X\n", 
                  filename, _total_size, _total_chunks, target_id);

    // Calcolo CRC32 (opzionale se il file è grande, ma richiesto dal protocollo)
    _full_crc32 = calculateFileCrc32();
    _file.seek(0); // Torna all'inizio

    _state = OtaSenderState::WAIT_START_ACK;
    sendStart();
    _last_action_ms = millis();
    return true;
}

void OtaSender::sendStart() {
    PayloadOtaStart start;
    memset(&start, 0, sizeof(PayloadOtaStart));
    start.ota_type = (uint8_t)_type;
    start.total_size = _total_size;
    start.chunk_size = 128;
    start.total_chunks = _total_chunks;
    start.crc32_full = _full_crc32;
    strncpy(start.version, "1.0.0", 8);

    LmpFrame frame;
    uint16_t len = lmp_build_frame(&frame, _target_id, LMP_ADDR_MASTER, MSG_OTA_START, loraManager.getNextSeq(), &start, sizeof(PayloadOtaStart));

    if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
        loraManager.sendRaw((uint8_t*)&frame, len);
        xSemaphoreGive(radioMutex);
    }
}

void OtaSender::sendCurrentChunk() {
    _file.seek(_current_chunk * 128);
    
    PayloadOtaChunk chunk;
    memset(&chunk, 0, sizeof(PayloadOtaChunk));
    chunk.chunk_index = _current_chunk;
    chunk.data_len = _file.read(chunk.data, 128);

    LmpFrame frame;
    uint16_t len = lmp_build_frame(&frame, _target_id, LMP_ADDR_MASTER, MSG_OTA_CHUNK, loraManager.getNextSeq(), &chunk, sizeof(PayloadOtaChunk));

    if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
        loraManager.sendRaw((uint8_t*)&frame, len);
        xSemaphoreGive(radioMutex);
    }
    Serial.printf("[OTA] Inviato chunk %d/%d\n", _current_chunk + 1, _total_chunks);
}

void OtaSender::sendEnd() {
    PayloadOtaEnd end;
    end.final_crc32 = _full_crc32;

    LmpFrame frame;
    uint16_t len = lmp_build_frame(&frame, _target_id, LMP_ADDR_MASTER, MSG_OTA_END, loraManager.getNextSeq(), &end, sizeof(PayloadOtaEnd));

    if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
        loraManager.sendRaw((uint8_t*)&frame, len);
        xSemaphoreGive(radioMutex);
    }
}

void OtaSender::handleOtaAck(const PayloadOtaAck* ack) {
    if (_state == OtaSenderState::WAIT_START_ACK && ack->status == ACK_OTA_READY) {
        Serial.println("[OTA] Slave pronto. Inizio invio chunk.");
        _state = OtaSenderState::SENDING_CHUNKS;
        _current_chunk = 0;
        _retries = 0;
        sendCurrentChunk();
        _state = OtaSenderState::WAIT_CHUNK_ACK;
        _last_action_ms = millis();
    }
    else if (_state == OtaSenderState::WAIT_CHUNK_ACK && ack->status == ACK_OTA_CHUNK_OK) {
        if (ack->chunk_index == _current_chunk) {
            _current_chunk++;
            _retries = 0;
            if (_current_chunk < _total_chunks) {
                sendCurrentChunk();
                _last_action_ms = millis();
            } else {
                Serial.println("[OTA] Tutti i chunk inviati. Invio fine OTA.");
                sendEnd();
                _state = OtaSenderState::WAIT_END_ACK;
                _last_action_ms = millis();
            }
        }
    }
    else if (_state == OtaSenderState::WAIT_END_ACK && (ack->status == ACK_OTA_DONE || ack->status == ACK_OTA_APPLYING)) {
        Serial.println("[OTA] Trasferimento completato con successo!");
        _state = OtaSenderState::SUCCESS;
        _file.close();
    }
    else if (ack->status == ACK_OTA_FAIL || ack->status == ACK_ERROR) {
        Serial.printf("[OTA] Fallimento riportato dallo slave: %s\n", ack->message);
        _state = OtaSenderState::FAILED;
        _file.close();
    }
}

void OtaSender::process() {
    if (_state == OtaSenderState::IDLE || _state == OtaSenderState::SUCCESS || _state == OtaSenderState::FAILED) return;

    if (millis() - _last_action_ms > ACK_TIMEOUT_MS) {
        if (_retries < ACK_RETRIES) {
            _retries++;
            Serial.printf("[OTA] Timeout ACK. Retry %d/%d...\n", _retries, ACK_RETRIES);
            if (_state == OtaSenderState::WAIT_START_ACK) sendStart();
            else if (_state == OtaSenderState::WAIT_CHUNK_ACK) sendCurrentChunk();
            else if (_state == OtaSenderState::WAIT_END_ACK) sendEnd();
            _last_action_ms = millis();
        } else {
            Serial.println("[OTA] Errore: Raggiunto limite retry. OTA fallito.");
            _state = OtaSenderState::FAILED;
            _file.close();
        }
    }
}

uint8_t OtaSender::getProgress() const {
    if (_total_chunks == 0) return 0;
    return (_current_chunk * 100) / _total_chunks;
}

void OtaSender::reset() {
    if (_file) _file.close();
    _state = OtaSenderState::IDLE;
}

uint32_t OtaSender::calculateFileCrc32() {
    // Implementazione semplificata o usa libreria CRC32
    // Per ora restituiamo un placeholder o implementiamo un loop
    uint32_t crc = 0xFFFFFFFF;
    _file.seek(0);
    while (_file.available()) {
        uint8_t b = _file.read();
        crc ^= b;
        for (int i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}
