#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 6); // RX, TX
String dataPackage;
float humidity = 10;
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  while (!Serial);
//  digitalWrite(20, HIGH);
//  delay(500);
//  digitalWrite(20, LOW); 
//  delay(2000);
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 delay(1000);
 LoRa.setSpreadingFactor(7);
 LoRa.setSignalBandwidth(62.5E3);
 LoRa.crc();
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) { // read byte by byte
      dataPackage = (char)LoRa.read();
      Serial.print(dataPackage);
      mySerial.print(dataPackage);
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    delay(50);

//    for (int i = 0; i < dataPackage.length(); i++)
//  {
//    mySerial.write(dataPackage[i]);   // Push each char 1 by 1 on each loop pass
//  }
    delay(50);
  }

}
