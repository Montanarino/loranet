#include "console.h"
#include "ota_sender.h"
#include "composer/module_composer.h"

// Referenze esterne definite in master.ino
extern LoRaManager loraManager;
extern SlaveRegistry slaveRegistry;
extern OtaSender otaSender;
extern SemaphoreHandle_t registryMutex;
extern SemaphoreHandle_t radioMutex;

ModuleComposer composer;


void console_init() {
    Serial.println("\nLMP CLI Console pronta. Digita 'help' per i comandi.");
    console_print_prompt();
}

void console_print_prompt() {
    Serial.print("\nLMP-MASTER> ");
}

void consoleTask(void *pvParameters) {
    (void)pvParameters;
    
    for (;;) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();

            if (input.length() > 0) {
                if (input == "list") {
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        uint8_t count = slaveRegistry.getRegisteredCount();
                        Serial.printf("\n--- TABELLA DISPOSITIVI LMP (Totale: %d) ---\n", count);
                        Serial.println("ID   | Nome Dispositivo | Stato   | Relay? | Last Seen");
                        Serial.println("-------------------------------------------------------");
                        
                        for (uint16_t i = 1; i <= 254; i++) {
                            SlaveNode* n = slaveRegistry.getSlaveById(i);
                            if (n != nullptr) {
                                uint32_t ago = (millis() - n->last_seen) / 1000;
                                Serial.printf("0x%02X | %-16s | %-7s | %-6s | %ds fa\n",
                                              n->id, n->name, n->is_online ? "ONLINE" : "OFFLINE",
                                              n->is_repeater ? "SI" : "NO", ago);
                            }
                        }
                        Serial.println("-------------------------------------------------------");
                        xSemaphoreGive(registryMutex);
                    }
                } 
                else if (input.startsWith("cmd ")) {
                    int t_id, s_id, c_id, a_val;
                    if (sscanf(input.c_str(), "cmd %d %d %d %d", &t_id, &s_id, &c_id, &a_val) == 4) {
                        PayloadCmd cmdPayload;
                        memset(&cmdPayload, 0, sizeof(PayloadCmd));
                        cmdPayload.service_id = (uint8_t)s_id; 
                        cmdPayload.cmd_id = (uint8_t)c_id;         
                        cmdPayload.args_len = 1;
                        cmdPayload.args[0] = (uint8_t)a_val;       

                        LmpFrame cmdFrame;
                        uint16_t cmd_len = lmp_build_frame(
                            &cmdFrame, (uint8_t)t_id, LMP_ADDR_MASTER, MSG_CMD,
                            loraManager.getNextSeq(), &cmdPayload, sizeof(PayloadCmd)
                        );

                        if (cmd_len > 0) {
                            if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
                                loraManager.sendRaw((uint8_t*)&cmdFrame, cmd_len);
                                xSemaphoreGive(radioMutex);
                            }
                            Serial.printf("[TX] Comando inviato! Nodo:0x%02X, Svc:0x%02X, Cmd:0x%02X, Arg:%d\n", 
                                          t_id, s_id, c_id, a_val);
                        }
                    } else {
                        Serial.println("Errore. Sintassi: cmd <NODO> <SERVIZIO> <COMANDO> <ARG>");
                    }
                }
                else if (input.startsWith("info ")) {
                    uint8_t target_id = (uint8_t)input.substring(5).toInt();
                    if (xSemaphoreTake(registryMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        SlaveNode* n = slaveRegistry.getSlaveById(target_id);
                        if (n != nullptr) {
                            Serial.printf("\n--- INFO NODO 0x%02X (%s) ---\n", n->id, n->name);
                            for (int i = 0; i < n->service_count; i++) {
                                Serial.printf(" [%d] ID: 0x%02X | %-15s | Ver: %d | Stato: %s\n",
                                              i+1, n->services[i].service_id, n->services[i].name, 
                                              n->services[i].version, n->services[i].active ? "ON" : "OFF");
                            }
                        } else {
                            Serial.printf("Errore: Nodo 0x%02X non trovato.\n", target_id);
                        }
                        xSemaphoreGive(registryMutex);
                    }
                }
                else if (input.startsWith("ota ")) {
                    int target_id;
                    char filename[64];
                    if (sscanf(input.c_str(), "ota %d %s", &target_id, filename) == 2) {
                        if (!otaSender.begin((uint8_t)target_id, filename, OTA_TYPE_FIRMWARE)) {
                            Serial.println("[CONSOLE] Impossibile avviare OTA. Verifica file e stato.");
                        }
                    } else {
                        Serial.println("Sintassi: ota <ID_NODO> <NOME_FILE>");
                    }
                }
                else if (input == "reset_ota") {
                    otaSender.reset();
                    Serial.println("[CONSOLE] OtaSender resettato.");
                }
                else if (input.startsWith("compose ")) {
                    int target_id, service_id;
                    if (sscanf(input.c_str(), "compose %d %d", &target_id, &service_id) == 2) {
                        PayloadConfig cfg;
                        composer.composeConfig((uint8_t)service_id, &cfg);
                        
                        LmpFrame frame;
                        uint16_t len = lmp_build_frame(&frame, (uint8_t)target_id, LMP_ADDR_MASTER, MSG_CONFIG, loraManager.getNextSeq(), &cfg, sizeof(PayloadConfig));
                        
                        if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
                            loraManager.sendRaw((uint8_t*)&frame, len);
                            xSemaphoreGive(radioMutex);
                            Serial.printf("[TX] Configurazione inviata al Nodo 0x%02X per Svc 0x%02X\n", target_id, service_id);
                        }
                    } else {
                        Serial.println("Sintassi: compose <ID_NODO> <ID_SERVIZIO>");
                    }
                }
                else if (input.startsWith("wifi ")) {
                    int target_id;
                    char ssid[20], pass[20];
                    int n = sscanf(input.c_str(), "wifi %d %19s %19s", &target_id, ssid, pass);
                    if (n >= 2) {
                        PayloadConfig cfg;
                        memset(&cfg, 0, sizeof(PayloadConfig));
                        cfg.service_id = SVC_HTTP_SERVER;
                        cfg.param_count = 2;
                        strncpy(cfg.params[0].key, "ssid", 12);
                        strncpy(cfg.params[0].value, ssid, 20);
                        strncpy(cfg.params[1].key, "pass", 12);
                        if (n == 3) strncpy(cfg.params[1].value, pass, 20);
                        else cfg.params[1].value[0] = '\0';

                        LmpFrame frame;
                        uint16_t len = lmp_build_frame(&frame, (uint8_t)target_id, LMP_ADDR_MASTER, MSG_CONFIG, loraManager.getNextSeq(), &cfg, sizeof(PayloadConfig));
                        
                        if (xSemaphoreTake(radioMutex, portMAX_DELAY) == pdTRUE) {
                            loraManager.sendRaw((uint8_t*)&frame, len);
                            xSemaphoreGive(radioMutex);
                            Serial.printf("[TX] Config WiFi inviata a 0x%02X (SSID: %s)\n", target_id, ssid);
                        }
                    } else {
                        Serial.println("Sintassi: wifi <id> <ssid> [pass]");
                    }
                }
                else if (input == "help") {
                    Serial.println("\nComandi CLI Disponibili:");
                    Serial.println("  list       - Mostra lo stato di tutti i nodi slave");
                    Serial.println("  info <id>  - Lista dei servizi di un nodo");
                    Serial.println("  cmd <n> <s> <c> <a> - Invia comando raw");
                    Serial.println("  wifi <id> <ssid> <pass> - Configura WiFi nodo");
                    Serial.println("  ota <id> <file>     - Avvia update OTA firmware");
                    Serial.println("  compose <id> <svc>  - Invia configurazione modulo");
                    Serial.println("  reset_ota  - Forza reset del modulo OTA");
                    Serial.println("  help       - Mostra questo menu");
                } 
                else {
                    Serial.println("Comando sconosciuto. Digita 'help' per la lista.");
                }
                console_print_prompt();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
