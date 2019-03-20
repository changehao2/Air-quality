#include <SPI.h>
#include <LoRa.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define sData 50
LiquidCrystal_I2C lcd(0x3F, 20, 4); // 0x27 for 1602 :)))
String dataPackage;

String h, t, c, dust;
char temp[sData];
// char* temp;
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 delay(1000);
 LoRa.setSpreadingFactor(8);
 LoRa.setSignalBandwidth(62.5E3);
 LoRa.crc();

 lcd.begin();
 lcd.backlight();
 lcd.setCursor(0,0); 
 lcd.print("Temperature: "); //13
 lcd.setCursor(0,1); 
 lcd.print("Humidity: "); 
 lcd.setCursor(0,2); 
 lcd.print("CO: ");
 lcd.setCursor(0,3); 
 lcd.print("Dust density: ");
// lcd.print("Hello, world!");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  int i = 0, k = 0;
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
       // dataPackage = (char)LoRa.read();
       // Serial.print(dataPackage); 
     temp[i] = (char)LoRa.read();
     ++i;
      // h = LoRa.readStringUntil(); // for fun :)))
      // *(temp + i) = (char)LoRa.read();
      // ++i;
    }
   for(int j = 0; j < sData; j++){
         Serial.print(temp[j]);
         if (temp[j] == ' ') break;
   }
     // Serial.print(temp);
//      h = Serial.readStringUntil('\n');
//      t = Serial.readStringUntil('\n');
//      c = Serial.readStringUntil('\n');
//      dust = Serial.readStringUntil('\n');

//LCD display
   lcd.setCursor(13,0);
   for(; k <sData; k++){
    if (temp[k] == '\n') break;
    lcd.print(temp[k]);
   }
   lcd.setCursor(13,1);
   for(k +=1; k <sData; k++){
    if (temp[k] == '\n') break;
    lcd.print(temp[k]);
   }
      lcd.setCursor(13,2);
   for(k +=1; k <sData; k++){
    if (temp[k] == '\n') break;
    lcd.print(temp[k]);
   }
      lcd.setCursor(13,3);
   for(k +=1; k <sData; k++){
    if (temp[k] == '\n') break;
    lcd.print(temp[k]);
   }
//      lcd.setCursor(13,0); 
//      lcd.print(t); 
//      lcd.setCursor(13,1); 
//      lcd.print(h); 
//      lcd.setCursor(13,2); 
//      lcd.print(c); 
//      lcd.setCursor(13,3); 
//      lcd.print(dust);          
    // print RSSI of packet
//    Serial.print("' with RSSI ");
//    Serial.println(LoRa.packetRssi());
  }
  // Clear data array_ 
     for(int j = 0; j < sData; j++){
       temp[j] = ' ';
   }
}























