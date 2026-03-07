#include <Arduino.h>
#include "LoRaModule.cpp"

// put function declarations here:
LoRaModule lora(5, 6, 9600);
void loraSetup();
void onTx();
void onRx(uint8_t*,uint16_t);
void onRxTimeout();
const uint8_t dataExpected[] = {'c', 'i', 'a', 'o'};

void setup() {

  lora.onTxDone(onTx);
  lora.onRxDone(onRx);
  lora.onRxTimeout(onRxTimeout);
  lora.setExpect(dataExpected, 5);
}

void loop() {
  // put your main code here, to run repeatedly:
  lora.poll();
}

// put function definitions here:
void loraSetup(){

}
void onTx(){
  printf("Inviato con successo, inizio ascolto...\n");
  lora.startReceive(1000);
}
void onRx(uint8_t* d,uint16_t l){
  printf("Ricevuto");
  for(int i = 0; i < l; i++){
    printf("%c", d[i]);
  }
  printf("\n");
}
void onRxTimeout(){
  
}