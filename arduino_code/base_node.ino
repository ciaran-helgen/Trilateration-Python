#include <SPI.h>
#include <LoRa.h>

//#define USE_SERIAL //uncomment to enable serial readout. Code will block until serial is available

int counter = 0;
int header = 0xAA; //header for this pair of nodes

int msg_rssi = 0;

int rxHeader = 0x00; // header of incoming message

// how often to ping the drone node
int sendInterval = 200;
// timestamp of last message sent
int lastSendTime = 0;

int rxCounter = 0;

void setup() {
  #ifdef USE_SERIAL
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");
  #endif

  while (!LoRa.begin(868E6)) {
    #ifdef USE_SERIAL
    Serial.println("Starting LoRa failed!");
    Serial.println("Retrying...");
    #endif
    delay(500);
  }
}

void loop() {
  if (millis() - lastSendTime > sendInterval) {
    sendPacket();
    //Serial.print("Sending ");
    //Serial.println(counter);
    lastSendTime = millis();
    counter++;
  }
}

void sendPacket() {
  //Serial.print("Sending packet: ");
  //Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.write(header);
  LoRa.write(counter);
  LoRa.endPacket();
  counter++;

}