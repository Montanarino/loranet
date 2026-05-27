#ifndef MODULE_COMPOSER_H
#define MODULE_COMPOSER_H

#include <Arduino.h>
#include "lora_protocol.h"

class ModuleComposer {
public:
    ModuleComposer() {}

    /**
     * @brief Genera una configurazione fittizia per un servizio.
     * In un caso reale leggerebbe dei template file.
     */
    void composeConfig(uint8_t service_id, PayloadConfig* out_config) {
        memset(out_config, 0, sizeof(PayloadConfig));
        out_config->service_id = service_id;
        
        if (service_id == SVC_SENSOR) {
            out_config->param_count = 2;
            strncpy(out_config->params[0].key, "interval", 12);
            strncpy(out_config->params[0].value, "5000", 20);
            strncpy(out_config->params[1].key, "pin", 12);
            strncpy(out_config->params[1].value, "34", 20);
        }
        else if (service_id == SVC_HTTP_SERVER) {
            out_config->param_count = 2;
            strncpy(out_config->params[0].key, "ssid", 12);
            strncpy(out_config->params[0].value, "IL_TUO_WIFI", 20); // Placeholder
            strncpy(out_config->params[1].key, "pass", 12);
            strncpy(out_config->params[1].value, "TUA_PASSWORD", 20); // Placeholder
        }
    }
};

#endif
