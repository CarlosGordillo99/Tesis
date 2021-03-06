#include <SPI.h>
#include <LoRa.h>
const int csPin = 16;
const int resetPin = 27;
const int irqPin = 16;

byte localAddress = 0xAA;
byte destinationAddress = 0xBB;
byte destination2Address = 0xCC;
long lastSendTime = 0;
int interval = 3000;
int count = 0;

void setup() {
  Serial.begin(115200);
  pinMode(17,OUTPUT);
  Serial.println("Start LoRa duplex");

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true) {}
  }
}

void loop() {
   if (millis() - lastSendTime > interval) {
    String sensor = String(count++);
    sendMessage(sensor);
    sendMessage2(sensor);

/*    Serial.print("Sending data " + sensorData);
    Serial.print(" from 0x" + String(localAddress, HEX));
    Serial.print(" to 0x" + String(destinationAddress, HEX));
    Serial.println(" to 0x" + String(destination2Address, HEX));*/

    lastSendTime = millis();
    interval = random(3000) + 1000;
  }

  receiveMessage(LoRa.parsePacket());
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destinationAddress);
  LoRa.write(localAddress);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
}
void sendMessage2(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destination2Address);
  LoRa.write(localAddress);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
}
void receiveMessage(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingLength = LoRa.read();

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (incomingLength != incoming.length()) {
  //  Serial.println("Error: Message length does not match length");
    return;
  }
  if (recipient != localAddress) {
   // Serial.println("Error: Recipient address does not match local address");
    return;
  }
  digitalWrite(17,HIGH);
  delay(200);
  digitalWrite(17,LOW);
  delay(200);
  Serial.println(incoming);
  
}
