#include <SparkFun_VEML6075_Arduino_Library.h>
VEML6075 uv;
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" 
SCD30 airSensor;
#include "SparkFun_Ublox_Arduino_Library.h" 
SFE_UBLOX_GPS myGPS;
#include <SPI.h>
#include <LoRa.h>
const int csPin = 16;
const int resetPin = 27;
const int irqPin = 16;
byte localAddress = 0xBB;
byte destinationAddress = 0xAA;
long lastSendTime = 0;
int interval = 2000;
int count = 0;
String coma=",";
String inputString = "";

void setup() {
  
  Serial.begin(115200);
  pinMode(17,OUTPUT);
  Wire.begin();
  Serial.println("Start LoRa duplex");

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true) {}
  }
    if (myGPS.begin() == false){
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
    }
      if (airSensor.begin() == false){
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    while (1);
  }

      if (uv.begin() == false) {
    Serial.println("Unable to communicate with VEML6075.");
    while (1) ;
  }
   myGPS.setI2COutput(COM_TYPE_UBX); 
   myGPS.saveConfiguration();

}

void loop() {
  long latitude = myGPS.getLatitude();
  long longitude = myGPS.getLongitude();
  long altitude = myGPS.getAltitude();
  if (Serial.available() > 0) {
   char dato = Serial.read();
   if(dato=='E'){
 //if (millis() - lastSendTime > interval) {
    String temperatura = String (airSensor.getTemperature(), 1);
    String humedad = String (airSensor.getHumidity(),1);
    String sensorData = (uv.index()+coma+airSensor.getCO2()+coma+temperatura+coma+humedad+coma+latitude+coma+longitude+coma+altitude+coma+"B");
    sendMessage(sensorData);
   /* Serial.print(sensorData);
    Serial.print(" from 0x" + String(localAddress, HEX));
    Serial.println(" to 0x" + String(destinationAddress, HEX));*/

    lastSendTime = millis();
    interval = random(2000) + 1000;
 // }
  receiveMessage(LoRa.parsePacket());

   }
  }
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destinationAddress);
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
 // Serial.println("Error: Message length does not match length");
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
  Serial.println('R');
}
