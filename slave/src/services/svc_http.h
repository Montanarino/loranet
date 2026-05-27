#ifndef SVC_HTTP_H
#define SVC_HTTP_H

#include "service.h"
#include <WebServer.h>

class HttpService : public Service {
private:
    uint16_t _port;
    WebServer* _server;
    
    void handleRoot();

public:
    HttpService(uint8_t id, const char* name, uint8_t version, uint16_t port = 80)
        : Service(id, name, version), _port(port), _server(nullptr) {}

    void init() override;
    void loop() override;
    void onConfigChanged() override;
    bool executeCmd(const PayloadCmd* cmd, PayloadAck* out_ack, PayloadCmdResult* out_result) override;

private:
    void connect();
};

#endif
