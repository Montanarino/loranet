
void setup() {
    Serial.begin(115200);
    
    frame_queue    = xQueueCreate(16, sizeof(LoRaFrame));
    registry_mutex = xSemaphoreCreateMutex();

    lora_manager.begin();   // configura modulo via AT

    // Lancia i 3 task
    xTaskCreatePinnedToCore(loraRxTask,    "lora_rx",   4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(mainLogicTask, "logic",     8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(consoleTask,   "console",   4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);  // loop non serve, i task fanno tutto
}