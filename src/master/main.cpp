#include <Arduino.h>
#include "LoRaModule.cpp"

// put function declarations here:
LoRaModule* lora;
void loraSetup();
void onTx();
void onRx(uint8_t*,uint16_t);
void onRxTimeout();
const uint8_t dataExpected[] = {'c', 'i', 'a', 'o'};
const uint8_t data[] = {'c', 'i', 'a', 'o'};

void setup() {
  lora = new LoRaModule(4, 5, 9600);
  lora->onTxDone(onTx);
  lora->onRxDone(onRx);
  lora->onRxTimeout(onRxTimeout);
  lora->setExpect(dataExpected, 5);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  lora->poll();
}

// put function definitions here:
void loraSetup(){

}
void onTx(){
  Serial.print("Inviato con successo, inizio ascolto...\n");
  lora->startReceive(1000);
}
void onRx(uint8_t* d,uint16_t l){
  Serial.print("Ricevuto: ");
  for(int i = 0; i < l; i++){
    Serial.print(d[i]);
  }
  Serial.print("\n");
}
void onRxTimeout(){
  //lora.send(data, sizeof(data))
}