#ifndef OTA_RECEIVER_H
#define OTA_RECEIVER_H

#include <Arduino.h>
#include <Update.h>
#include "lora_protocol.h"

class OtaReceiver {
private:
    bool _is_ongoing;
    OtaType _type;
    uint32_t _total_size;
    uint16_t _total_chunks;
    uint16_t _next_chunk_idx;
    uint32_t _crc32_full;
    uint32_t _received_size;

public:
    OtaReceiver();
    void handleStart(const PayloadOtaStart* start, PayloadOtaAck* out_ack);
    void handleChunk(const PayloadOtaChunk* chunk, PayloadOtaAck* out_ack);
    void handleEnd(const PayloadOtaEnd* end, PayloadOtaAck* out_ack);
    
    bool isOngoing() const { return _is_ongoing; }
    void reset();
};

#endif
