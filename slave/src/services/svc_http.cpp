#include "svc_http.h"
#include <WiFi.h>
#include <Preferences.h>

void HttpService::init() {
    _is_active = false;
    connect();
}

void HttpService::onConfigChanged() {
    Serial.println("[HTTP] Configurazione ricevuta, ricollego Wi-Fi...");
    connect();
}

void HttpService::connect() {
    // Prova a caricare le credenziali Wi-Fi salvate in NVS
    Preferences prefs;
    prefs.begin("lmp_cfg", true);
    String ssid = prefs.getString("s2_ssid", "");
    String pass = prefs.getString("s2_pass", "");
    prefs.end();

    if (ssid.length() > 0) {
        Serial.printf("[HTTP] Connessione a %s...\n", ssid.c_str());
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        // Non blocchiamo l'init, il controllo avverrà nel loop o allo START
    } else {
        Serial.println("[HTTP] Nessun SSID configurato.");
    }
}

void HttpService::loop() {
    if (_server && WiFi.status() == WL_CONNECTED) {
        _server->handleClient();
    }
}

void HttpService::handleRoot() {
    String html = "<html><head><title>LMP Node</title></head><body>";
    html += "<h1>LMP Slave: " + String(_name) + "</h1>";
    html += "<p>IP Address: " + WiFi.localIP().toString() + "</p>";
    html += "<p>Uptime: " + String(millis()/1000) + "s</p>";
    html += "<p>RSSI Wi-Fi: " + String(WiFi.RSSI()) + " dBm</p>";
    html += "</body></html>";
    _server->send(200, "text/html", html);
}

bool HttpService::executeCmd(const PayloadCmd* cmd, PayloadAck* out_ack, PayloadCmdResult* out_result) {
    (void)out_result;

    if (cmd->cmd_id == 0x01) { // START SERVER
        if (WiFi.status() != WL_CONNECTED) {
            out_ack->status = ACK_ERROR;
            strncpy(out_ack->message, "NO WIFI", 8);
            return false;
        }

        if (!_server) {
            _server = new WebServer(_port);
            _server->on("/", std::bind(&HttpService::handleRoot, this));
            _server->begin();
            _is_active = true;
            Serial.printf("[HTTP] Server attivo su http://%s\n", WiFi.localIP().toString().c_str());
        }
        out_ack->status = ACK_OK;
        strncpy(out_ack->message, "STARTED", 8);
        return true;
    }
    
    if (cmd->cmd_id == 0x02) { // STOP SERVER
        if (_server) {
            _server->stop();
            delete _server;
            _server = nullptr;
            _is_active = false;
        }
        out_ack->status = ACK_OK;
        strncpy(out_ack->message, "STOPPED", 8);
        return true;
    }

    out_ack->status = ACK_UNKNOWN_CMD;
    return false;
}
