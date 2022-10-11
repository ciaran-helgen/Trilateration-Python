#include <SPI.h>
#include <LoRa.h>

int header = 0xAA; //header for this pair of nodes

int msg_rssi = 0;

int rxHeader = 0x00; //header of incoming message

// how long to wait (millis) between received messages (avoid reflected duplicates)
int rxMinInterval = 1;
// millis timestamp of last message received
int lastRxTime = 0;

int rxCounter = 0;
int nextCounter = 0;
 
unsigned int distance_cm;

void setup() {
  
  Serial.begin(115200);
  while (!Serial); //wait for serial to connect

  while (!LoRa.begin(868E6)) {delay(500); } //loop until LoRa initialises
}

void loop() {
  if (LoRa.parsePacket()) {
    receivePacket();
    //only print if it has been rxMinInterval ms since last packet
    if (header == rxHeader && (millis() - lastRxTime > rxMinInterval)) {
       Serial.print(String(LoRa.packetRssi())+ '\n');
       lastRxTime = millis();
    }
  }
}

void receivePacket() {
  //Serial.println("at receive :" + (String)cycles);
  rxHeader = (int)LoRa.read();
  rxCounter = (int)LoRa.read();
  msg_rssi = LoRa.rssi();
  //Serial.print("received header: ");
  //Serial.println(rxHeader, HEX);

}