#ifndef OTA_SENDER_H
#define OTA_SENDER_H

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include "lora_protocol.h"
#include "lora_manager.h"

enum class OtaSenderState {
    IDLE,
    WAIT_START_ACK,
    SENDING_CHUNKS,
    WAIT_CHUNK_ACK,
    WAIT_END_ACK,
    SUCCESS,
    FAILED
};

class OtaSender {
private:
    uint8_t _target_id;
    OtaType _type;
    File    _file;
    
    uint32_t _total_size;
    uint16_t _total_chunks;
    uint16_t _current_chunk;
    uint32_t _full_crc32;
    
    OtaSenderState _state;
    uint32_t _last_action_ms;
    uint8_t  _retries;
    
    void sendStart();
    void sendCurrentChunk();
    void sendEnd();
    uint32_t calculateFileCrc32();

public:
    OtaSender();
    bool begin(uint8_t target_id, const char* filename, OtaType type);
    void process(); // Chiamata periodica dal task
    void handleOtaAck(const PayloadOtaAck* ack);
    
    OtaSenderState getState() const { return _state; }
    uint8_t getProgress() const;
    void reset();
};

#endif
