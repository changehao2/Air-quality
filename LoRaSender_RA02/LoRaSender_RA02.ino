// FOR ARDUINO MEGA :))
#include <SPI.h>
#include <LoRa.h>

int counter = 0;
String dataPackage = "";
float h = 38;
float t = 40;
float c = 11.2;
float dust = 300.03;

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
//  digitalWrite(9, HIGH);
//  delay(500);
//  digitalWrite(9, LOW); 
//  delay(2000);
  
  LoRa.setPins(53, 9, 2);
  while (!Serial);
//SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));  Serial.println("LoRa Sender");
//  digitalWrite(12, HIGH);
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
  Serial.print("Sending packet: ");
  Serial.println(counter);
  dataPackage = String(h) + "\n" + String(t) + "\n" + String(c) + "\n" + String(dust) + "\n";
  // send packet
  LoRa.beginPacket();
  // LoRa.print("Start: ");
  LoRa.print(dataPackage);
  // LoRa.print(" end");
//  LoRa.write(dataPakage);
//  LoRa.write(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}




