#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 6); // RX, TX
String dataPackage;

// For test send data
//int counter = 0;
//String dataPackage = "";
//float h = 38.888;
//float t = 40;
//float c = 11.2;
//float dust = 300.03;

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
 LoRa.setSpreadingFactor(8);
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

/*
//Send data
  Serial.print("Sending packet: ");
  Serial.println(counter);
  dataPackage = String(h) + "\n" + String(t) + "\n" + String(c) + "\n" + String(dust) + "\n";
  
  // send packet
  LoRa.beginPacket();
  // LoRa.print("Start: ");
  LoRa.print(dataPackage); // Here bro
//LoRa.print("hello ");
//LoRa.write(counter);
  // LoRa.print(" end");
//  LoRa.write(dataPackage);
//  LoRa.write(counter);
  LoRa.endPacket();

  counter++;
*/
//  delay(5000);
}
